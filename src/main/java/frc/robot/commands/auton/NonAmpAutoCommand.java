package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class NonAmpAutoCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private DriveAutonCommand firstPath;
  private DriveAutonCommand secondPath;
  private DriveAutonCommand thirdPath;
  private DriveAutonCommand fourthPath;
  private boolean hasGenerated = false;

  public NonAmpAutoCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      String firstPathName,
      String secondPathName,
      String thirdPathName,
      String fourthPathName) {
    firstPath = new DriveAutonCommand(driveSubsystem, firstPathName, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, secondPathName, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, thirdPathName, true, false);
    fourthPath = new DriveAutonCommand(driveSubsystem, fourthPathName, true, false);

    addCommands(
        new SequentialCommandGroup(
            new ResetGyroCommand(driveSubsystem),
            new ParallelCommandGroup(
                new setAngleOffsetCommand(driveSubsystem, -50.0),
                new ZeroElbowCommand(elbowSubsystem)),
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
            firstPath,
            secondPath,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
            thirdPath,
            fourthPath,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)
            // new ToggleVisionUpdatesCommand(driveSubsystem)
            ));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    fourthPath.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
