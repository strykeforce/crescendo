package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;

public class newLeftAuto4PieceTurn extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fifthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public newLeftAuto4PieceTurn(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree,
      String pathFour) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, false, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, false, false);
    fourthPath = new DriveAutonCommand(driveSubsystem, pathFour, false, false);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ZeroGyroCommand(driveSubsystem),
        new setAngleOffsetCommand(driveSubsystem, 60.0),
        firstPath,
        secondPath,
        thirdPath,
        fourthPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    fourthPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
