package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;

public class newLeftAuto4Piece extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public newLeftAuto4Piece(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, true, false);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ZeroGyroCommand(driveSubsystem),
        new setAngleOffsetCommand(driveSubsystem, 60.0),
        firstPath,
        secondPath,
        thirdPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
