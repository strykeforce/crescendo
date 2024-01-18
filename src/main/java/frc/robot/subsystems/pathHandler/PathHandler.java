package frc.robot.subsystems.pathHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import java.util.ArrayList;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PathHandler extends MeasurableSubsystem {

  PathStates curState = PathStates.FETCH;
  DeadEyeSubsystem deadeye;
  ArrayList<Integer> noteOrder;
  PathData nextPath;
  PathData endPath;
  PathData lastReturnedPath;
  DriveSubsystem driveSubsystem;
  boolean useDeadeye;
  double numPieces;
  Logger logger;
  boolean canShoot = false;
  boolean handling = false;
  Timer timer = new Timer();
  Rotation2d robotHeading;
  Trajectory curTrajectory;

  //  NOTE NUMBERS ARE BASED ON THE STANDARDS DOCUMENT
  // A 6x6 array that stores all paths (0 is shootPos, 1 is first note, etc)
  // First index is the starting path and second index is ending path
  // Example [0][5] is a path that goes from shooting position to the fifth note
  // Example [1][0] is a path that goes from note one to shooting position
  // Example [1][2] is a path that goes from note one to note two
  PathData[][] paths;

  RobotStateSubsystem robotStateSubsystem;

  public PathHandler(
      DeadEyeSubsystem deadeye,
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      ArrayList<Integer> order,
      String[][] pathNames,
      boolean useDeadeye,
      double numPieces,
      String endPath) {
    this.deadeye = deadeye;
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.useDeadeye = useDeadeye;
    this.numPieces = numPieces - 1.0;
    noteOrder = order;

    logger = LoggerFactory.getLogger(this.getClass());

    paths = new PathData[6][6];
    noteOrder.add(0);

    for (int i : noteOrder)
        for (int j : noteOrder) 
            if (i != j) paths[i][j] = driveSubsystem.generateTrajectory(pathNames[i][j]);
    
    noteOrder.remove(noteOrder.indexOf(0));

    this.endPath = driveSubsystem.generateTrajectory(endPath);
  }

  public PathStates getState() {
    return curState;
  }

  public void setState(PathStates state) {
    curState = state;
  }

  public void setPreference(ArrayList<Integer> list) {
    noteOrder = list;
  }

  public boolean hasNewPath() {
    return nextPath != lastReturnedPath;
  }

  public void startHandling() {
    timer.reset();
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

    driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);
  }

  @Override
  public void periodic() {
    if (handling) {
      switch (curState) {
        case SHOOT:
          if (canShoot) {
            robotStateSubsystem.shoot();
            canShoot = false;
          }

          if (!robotStateSubsystem.hasGamePiece()) {
            numPieces -= 0.5;
            if (noteOrder.size() == 0 || numPieces < 0.01) {
              nextPath = endPath;
            } else {
              nextPath = paths[0][noteOrder.get(0)];
            }

            startNewPath(nextPath);
            logger.info("SHOOT -> DRIVE_FETCH");
            curState = PathStates.DRIVE_FETCH;
          }
          break;

        case FETCH:
          if (noteOrder.size() > 1) {
            nextPath = paths[noteOrder.get(0)][noteOrder.get(1)];
          } else nextPath = endPath;

          if (robotStateSubsystem.hasGamePiece() && numPieces > 0.51) {
            numPieces -= 0.5;
            logger.info("FETCH -> DRIVE_SHOOT");
            nextPath = paths[noteOrder.get(0)][0];
            noteOrder.remove(0);
            curState = PathStates.SHOOT;
            startNewPath(nextPath);
          }

          if (timer.hasElapsed(AutonConstants.delayForPickup)) {
            logger.info("FETCH -> DRIVE_FETCH");
            noteOrder.remove(0);
            curState = PathStates.DRIVE_FETCH;
            startNewPath(nextPath);
          }
          break;

        case DRIVE_FETCH:
          driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);
          
          if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
            logger.info("DRIVE_FETCH -> FETCH");
            timer.reset();
            curState = PathStates.FETCH;
            driveSubsystem.setEnableHolo(false);
            driveSubsystem.grapherTrajectoryActive(false);
          }

          break;
        case DRIVE_SHOOT:
          driveSubsystem.calculateController(curTrajectory.sample(timer.get()), robotHeading);

          if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
            logger.info("DRIVE_SHOOT -> SHOOT");
            timer.reset();
            curState = PathStates.SHOOT;
            startShot();
            driveSubsystem.setEnableHolo(false);
            driveSubsystem.grapherTrajectoryActive(false);
          }

          break;
      }
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }

  public enum PathStates {
    SHOOT,
    FETCH,
    DRIVE_FETCH,
    DRIVE_SHOOT
  }
}
