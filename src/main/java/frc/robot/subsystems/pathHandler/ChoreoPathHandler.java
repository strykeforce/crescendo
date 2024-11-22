package frc.robot.subsystems.pathHandler;

import choreo.Choreo;
import choreo.trajectory.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ChoreoPathHandler extends MeasurableSubsystem {

  private PathStates curState = PathStates.DONE;
  private DeadEyeSubsystem deadeye;
  private boolean useDeadeye;
  private ArrayList<Integer> noteOrder;
  private Trajectory<SwerveSample> nextPath;
  private Trajectory<SwerveSample> lastReturnedPath;
  private DriveSubsystem driveSubsystem;
  private double numPieces;
  private Logger logger;
  private boolean canShoot = false;
  private boolean handling = false;
  private Timer timer = new Timer();
  private Rotation2d robotHeading;
  private Trajectory<SwerveSample> curTrajectory;
  private boolean deadeyeFlag = false;
  private int lastNote = 0;
  private boolean mirrorTrajectory = false;

  private ProfiledPIDController deadeyeYDrive;
  private PIDController deadeyeXDrive;

  private LedSubsystem ledSubsystem;
  // NOTE NUMBERS ARE BASED ON THE STANDARDS DOCUMENT
  // A 6x6 array that stores all paths (0 is shootPos, 1 is first note, etc)
  // First index is the starting path and second index is ending path
  // Example [0][5] is a path that goes from shooting position to the fifth note
  // Example [1][0] is a path that goes from note one to shooting position
  // Example [1][2] is a path that goes from note one to note two
  private Trajectory<SwerveSample>[][] paths;
  private String[][] pathNames;
  private Pose2d shotLoc;

  private RobotStateSubsystem robotStateSubsystem;
  private boolean isSpinningUp = false;

  public ChoreoPathHandler(
      DeadEyeSubsystem deadeye,
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      LedSubsystem ledSubsystem,
      List<Integer> order,
      String[][] pathNames,
      boolean useDeadeye,
      double numPieces,
      Pose2d shotLoc) {
    this.deadeye = deadeye;
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.useDeadeye = useDeadeye;
    this.numPieces = numPieces - 1.0;
    this.pathNames = pathNames;
    this.shotLoc = shotLoc;
    this.ledSubsystem = ledSubsystem;
    noteOrder = new ArrayList<>(order);

    logger = LoggerFactory.getLogger(this.getClass());

    deadeyeYDrive =
        new ProfiledPIDController(
            AutonConstants.kPDeadEyeYDrive,
            AutonConstants.kIDeadEyeYDrive,
            AutonConstants.kDDeadEyeYDrive,
            new Constraints(
                AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
    deadeyeXDrive =
        new PIDController(
            AutonConstants.kPDeadEyeXDrive,
            AutonConstants.kIDeadEyeXDrive,
            AutonConstants.kDDeadEyeXDrive);
    generateTrajectory();
  }

  public void generateTrajectory() {
    noteOrder.add(0);

    Set<Integer> singleNotes = new HashSet<Integer>(noteOrder);

    for (int i : singleNotes)
      for (int j : singleNotes)
        if (i != j) {
          Optional<Trajectory<SwerveSample>> tempPath = Choreo.loadTrajectory(pathNames[i][j]);
          if (tempPath.isPresent()) {
            paths[i][j] = tempPath.get();
          }
        }
    ;

    noteOrder.remove(noteOrder.indexOf(0));

    mirrorTrajectory = driveSubsystem.shouldFlip();
  }

  public PathStates getState() {
    return curState;
  }

  public void setState(PathStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  public void setPreference(List<Integer> list) {
    noteOrder = new ArrayList<>(list);
  }

  public void setNumPieces(double numPieces) {
    logger.info("set numPieces to: {}", numPieces);
    this.numPieces = numPieces - 1;
  }

  public void setPaths(String[][] pathNames) {
    this.pathNames = pathNames;
  }

  public void setShotLoc(Pose2d shotLoc) {
    this.shotLoc = shotLoc;
  }

  public boolean hasNewPath() {
    return nextPath != lastReturnedPath;
  }

  public void startHandling() {
    timer.reset();
    timer.start();
    deadeye.setCamEnabled(true);
    setState(PathStates.FETCH);
    handling = true;
  }

  // Command calls this when drive path is done driving
  public void startShot() {
    canShoot = true;
  }

  public Trajectory<SwerveSample> getNextPath() {
    lastReturnedPath = nextPath;
    if (curState == PathStates.FETCH && noteOrder.size() > 1) noteOrder.remove(0);
    return nextPath;
  }

  public void killPathHandler() {
    driveSubsystem.move(0.0, 0.0, 0.0, true);
    setState(PathStates.DONE);
  }

  public void startNewPath(Trajectory<SwerveSample> path) {
    curTrajectory = path;
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    logger.info("Begin new path");
    timer.reset();
    timer.start();

    driveSubsystem.calculateController(curTrajectory.sampleAt(timer.get(), mirrorTrajectory));
  }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput("States/PathHandler State", curState.name());
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler/NumPieces", numPieces);
    org.littletonrobotics.junction.Logger.recordOutput(
        "PathHandler/noteOrderSize", noteOrder.size());
    if (handling) {
      switch (curState) {
        case SHOOT:
          if (canShoot) {
            robotStateSubsystem.startShoot();
            canShoot = false;
          }

          if (!robotStateSubsystem.hasNote()) {
            numPieces -= 0.5;
            if (noteOrder.size() == 0 || numPieces < 0.01) {
              curState = PathStates.DONE;
              break;
            } else {
              nextPath = paths[0][noteOrder.get(0)];
            }
            logger.info("" + noteOrder.toString());
            logger.info("Begin Trajectory " + pathNames[0][noteOrder.get(0)]);
            startNewPath(nextPath);
            logger.info("SHOOT -> DRIVE_FETCH");
            curState = PathStates.DRIVE_FETCH;
          }
          break;
        case DRIVE_FETCH:
          driveSubsystem.calculateController(curTrajectory.sampleAt(timer.get(), mirrorTrajectory));
          double curX = driveSubsystem.getPoseMeters().getX();

          if (!timer.hasElapsed(curTrajectory.getTotalTime() * 0.50)
              && robotStateSubsystem.hasNote()) {
            numPieces -= 0.5;
            logger.info("FETCH -> DRIVE_SHOOT");
            nextPath = paths[lastNote][0];
            logger.info("Begin Trajectory " + pathNames[lastNote][0]);
            curState = PathStates.DRIVE_SHOOT;
            isSpinningUp = false;
            startNewPath(nextPath);
          }

          if (timer.hasElapsed(curTrajectory.getTotalTime() * AutonConstants.kPercentLeft)
              && ((robotStateSubsystem.getAllianceColor() == Alliance.Blue
                      && curX >= AutonConstants.kSwitchXLine)
                  || (robotStateSubsystem.getAllianceColor() == Alliance.Red
                      && curX <= DriveConstants.kFieldMaxX - AutonConstants.kSwitchXLine))) {

            // driveSubsystem.drive(0, 0, 0);
            logger.info("DRIVE_FETCH -> END_PATH");
            driveSubsystem.setDeadEyeDrive(true);
            ledSubsystem.setColor(120, 38, 109);
            deadeyeYDrive =
                new ProfiledPIDController(
                    AutonConstants.kPDeadEyeYDrive,
                    AutonConstants.kIDeadEyeYDrive,
                    AutonConstants.kDDeadEyeYDrive,
                    new Constraints(
                        AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
            // timer.reset();
            // timer.start();
            curState = PathStates.END_PATH;
            // driveSubsystem.setEnableHolo(false);
            // driveSubsystem.grapherTrajectoryActive(false);
          }

          break;
        case END_PATH:
          double yVel = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
          driveSubsystem.recordYVel(yVel);
          if (deadeye.getNumTargets() > 0)
            driveSubsystem.driveAutonXController(
                curTrajectory.sampleAt(timer.get(), mirrorTrajectory), yVel);
          else
            driveSubsystem.calculateController(
                curTrajectory.sampleAt(timer.get(), mirrorTrajectory));

          if (robotStateSubsystem.hasNote()) {
            numPieces -= 0.5;
            logger.info("END_PATH -> DRIVE_SHOOT");
            logger.info("" + noteOrder.toString());
            nextPath = paths[noteOrder.get(0)][0];
            logger.info("Begin Trajectory " + pathNames[noteOrder.get(0)][0]);
            curState = PathStates.DRIVE_SHOOT;
            isSpinningUp = false;
            noteOrder.remove(0);
            startNewPath(nextPath);
          }
          if (timer.hasElapsed(curTrajectory.getTotalTime())) {
            driveSubsystem.drive(0, 0, 0);
            logger.info("END_PATH -> FETCH");
            driveSubsystem.setDeadEyeDrive(false);
            timer.reset();
            timer.start();
            curState = PathStates.FETCH;
            driveSubsystem.setEnableHolo(false);
            driveSubsystem.grapherTrajectoryActive(false);
          }
          break;
        case FETCH:
          if ((robotStateSubsystem.getAllianceColor() == Alliance.Blue
                  && driveSubsystem.getPoseMeters().getX()
                      <= DriveConstants.kFieldMaxX / 2 + AutonConstants.kMaxXOff)
              || (robotStateSubsystem.getAllianceColor() == Alliance.Red
                  && driveSubsystem.getPoseMeters().getX()
                      >= DriveConstants.kFieldMaxX / 2 - AutonConstants.kMaxXOff)) {
            if (deadeye.getNumTargets() > 0) {
              deadeyeFlag = true;
              double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
              double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
              driveSubsystem.move(
                  AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
            } else {
              driveSubsystem.move(0.0, 0.0, 0.0, true);
            }
          } else {
            driveSubsystem.move(0.0, 0.0, 0.0, true);
          }

          String nextPathName;

          if (robotStateSubsystem.hasNote() && numPieces > 0.51) {
            lastNote = noteOrder.get(0);
            numPieces -= 0.5;
            logger.info("FETCH -> DRIVE_SHOOT");
            logger.info("" + noteOrder.toString());
            deadeyeFlag = false;
            nextPath = paths[noteOrder.get(0)][0];
            logger.info("Begin Trajectory " + pathNames[noteOrder.get(0)][0]);
            noteOrder.remove(0);
            curState = PathStates.DRIVE_SHOOT;
            isSpinningUp = false;
            startNewPath(nextPath);
          }

          if (timer.hasElapsed(AutonConstants.kDelayForDeadeye)
              || deadeye.getNumTargets() == 0
                  && !deadeyeFlag
                  && timer.hasElapsed(AutonConstants.kDelayForPickup)) {
            deadeyeFlag = false;
            if (noteOrder.size() > 1) {
              nextPathName = pathNames[noteOrder.get(0)][noteOrder.get(1)];
              nextPath = paths[noteOrder.get(0)][noteOrder.get(1)];
            } else {
              curState = PathStates.DONE;
              break;
            }
            logger.info("" + noteOrder.toString());
            lastNote = noteOrder.get(0);
            logger.info("FETCH -> DRIVE_FETCH");
            logger.info("Begin Trajectory " + nextPathName);
            noteOrder.remove(0);
            curState = PathStates.DRIVE_FETCH;
            startNewPath(nextPath);
          }
          break;

        case DRIVE_SHOOT:
          driveSubsystem.calculateController(curTrajectory.sampleAt(timer.get(), mirrorTrajectory));

          if (!isSpinningUp
              && robotStateSubsystem.intakeHasNote()
              && robotStateSubsystem.magazineHasNote()) {
            robotStateSubsystem.spinUpShotSolution(shotLoc);
            isSpinningUp = true;
          }

          if (timer.hasElapsed(curTrajectory.getTotalTime())) {
            logger.info("DRIVE_SHOOT -> SHOOT");
            timer.reset();
            timer.start();
            curState = PathStates.SHOOT;
            startShot();
            driveSubsystem.setEnableHolo(false);
            driveSubsystem.move(0.0, 0.0, 0.0, false);
            driveSubsystem.grapherTrajectoryActive(false);
          }

          break;

        case DONE:
          handling = false;
          break;

        default:
          break;
      }
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  public enum PathStates {
    SHOOT,
    FETCH,
    END_PATH,
    DRIVE_FETCH,
    DRIVE_SHOOT,
    DONE
  }
}
