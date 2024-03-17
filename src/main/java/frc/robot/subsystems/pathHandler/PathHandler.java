package frc.robot.subsystems.pathHandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {

  private PathStates curState = PathStates.FETCH;
  private DeadEyeSubsystem deadeye;
  private boolean useDeadeye;
  private ArrayList<Integer> noteOrder;
  private PathData nextPath;
  private PathData lastReturnedPath;
  private DriveSubsystem driveSubsystem;
  private double numPieces;
  private Logger logger;
  private boolean canShoot = false;
  private boolean handling = false;
  private Timer timer = new Timer();
  private Rotation2d robotHeading;
  private Trajectory curTrajectory;

  // NOTE NUMBERS ARE BASED ON THE STANDARDS DOCUMENT
  // A 6x6 array that stores all paths (0 is shootPos, 1 is first note, etc)
  // First index is the starting path and second index is ending path
  // Example [0][5] is a path that goes from shooting position to the fifth note
  // Example [1][0] is a path that goes from note one to shooting position
  // Example [1][2] is a path that goes from note one to note two
  private PathData[][] paths = new PathData[6][6];
  private String[][] pathNames;
  private Pose2d shotLoc;

  private RobotStateSubsystem robotStateSubsystem;

  public PathHandler(
      DeadEyeSubsystem deadeye,
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
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
    noteOrder = new ArrayList<>(order);

    logger = LoggerFactory.getLogger(this.getClass());

    generateTrajectory();
  }

  public void generateTrajectory() {
    noteOrder.add(0);

    Set<Integer> singleNotes = new HashSet<Integer>(noteOrder);

    for (int i : singleNotes)
      for (int j : singleNotes)
        if (i != j) paths[i][j] = driveSubsystem.generateTrajectory(pathNames[i][j]);

    noteOrder.remove(noteOrder.indexOf(0));
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
    handling = true;
  }

  // Command calls this when drive path is done driving
  public void startShot() {
    canShoot = true;
  }

  public PathData getNextPath() {
    lastReturnedPath = nextPath;
    if (curState == PathStates.FETCH && noteOrder.size() > 1) noteOrder.remove(0);
    return nextPath;
  }

  public void startNewPath(PathData path) {
    curTrajectory = path.trajectory;
    robotHeading = path.targetYaw;
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    logger.info("Begin new path");
    timer.reset();
    timer.start();

    driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);
  }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler State", curState.name());
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

            logger.info("Begin Trajectory " + pathNames[0][noteOrder.get(0)]);
            startNewPath(nextPath);
            logger.info("SHOOT -> DRIVE_FETCH");
            curState = PathStates.DRIVE_FETCH;
          }
          break;

        case DRIVE_FETCH:
          driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);

          if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {

            driveSubsystem.drive(0, 0, 0);
            logger.info("DRIVE_FETCH -> FETCH");
            timer.reset();
            timer.start();
            curState = PathStates.FETCH;
            driveSubsystem.setEnableHolo(false);
            driveSubsystem.grapherTrajectoryActive(false);
          }

          break;

        case FETCH:
          if (noteOrder.size() > 1) {
            nextPath = paths[noteOrder.get(0)][noteOrder.get(1)];
          } else {
            curState = PathStates.DONE;
            break;
          }

          if (robotStateSubsystem.hasNote() && numPieces > 0.51) {
            numPieces -= 0.5;
            logger.info("FETCH -> DRIVE_SHOOT");
            nextPath = paths[noteOrder.get(0)][0];
            logger.info("Begin Trajectory " + pathNames[noteOrder.get(0)][0]);
            noteOrder.remove(0);
            curState = PathStates.DRIVE_SHOOT;
            startNewPath(nextPath);
          }

          if (timer.hasElapsed(AutonConstants.kDelayForPickup)) {
            logger.info("FETCH -> DRIVE_FETCH");
            noteOrder.remove(0);
            curState = PathStates.DRIVE_FETCH;
            startNewPath(nextPath);
          }
          break;

        case DRIVE_SHOOT:
          driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);

          if (robotStateSubsystem.intakeHasNote() && robotStateSubsystem.magazineHasNote()) {
            robotStateSubsystem.spinUpShotSolution(shotLoc);
          }

          if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
            logger.info("DRIVE_SHOOT -> SHOOT");
            timer.reset();
            timer.start();
            curState = PathStates.SHOOT;
            startShot();
            driveSubsystem.setEnableHolo(false);
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
    DRIVE_FETCH,
    DRIVE_SHOOT,
    DONE
  }
}
