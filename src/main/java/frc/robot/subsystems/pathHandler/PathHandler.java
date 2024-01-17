package frc.robot.subsystems.pathHandler;

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
  DriveSubsystem driveSubsystem;
  boolean useDeadeye;
  float numPieces;
  Logger logger;
  boolean canShoot = false;

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
      String[][] paths,
      boolean useDeadeye,
      float numPieces,
      String endPath) {
    this.deadeye = deadeye;
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.useDeadeye = useDeadeye;
    this.numPieces = numPieces;
    noteOrder = order;

    logger = LoggerFactory.getLogger(this.getClass());

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) {
        if (i != j) this.paths[i][j] = driveSubsystem.generateTrajectory(paths[i][j]);
      }

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

  public PathData getNextPath() {
    if (curState == PathStates.FETCH && noteOrder.size() > 1) noteOrder.remove(0);
    return nextPath;
  }

  //Command calls this when drive path is done driving
  public void startShot() {
    canShoot = true;
  }

  @Override
  public void periodic() {
    switch (curState) {
      case SHOOT:
        if (canShoot) {
          robotStateSubsystem.shoot();
          canShoot = false;
        }

        if (!robotStateSubsystem.hasGamePiece()) {
          logger.info("SHOOT -> FETCH");
          nextPath = paths[0][noteOrder.get(0)];
          curState = PathStates.FETCH;
        }
        break;

      case FETCH:
        if (noteOrder.size() > 1) nextPath = paths[noteOrder.get(0)][noteOrder.get(1)];
        else nextPath = endPath;

        if (robotStateSubsystem.hasGamePiece()) {
          logger.info("FETCH -> SHOOT");
          nextPath = paths[noteOrder.get(0)][0];
          curState = PathStates.SHOOT;
        }
        break;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }

  public enum PathStates {
    SHOOT,
    FETCH
  }
}
