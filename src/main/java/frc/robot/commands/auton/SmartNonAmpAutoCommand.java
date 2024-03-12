package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.pathHandler.PathHandlerShootCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import java.util.List;

public class SmartNonAmpAutoCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private PathHandler pathHandler;
  private boolean hasGenerated = false;

  public SmartNonAmpAutoCommand(
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

    pathHandler =
        new PathHandler(
            null,
            robotStateSubsystem,
            driveSubsystem,
            List.of(5, 4, 3, 2, 1),
            AutonConstants.kNonAmpPathMatrix,
            false,
            2.0);
    addCommands(
        new SequentialCommandGroup(
            new ResetGyroCommand(driveSubsystem),
            new ParallelCommandGroup(
                new setAngleOffsetCommand(driveSubsystem, -50.0),
                new ZeroElbowCommand(elbowSubsystem)),
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
            new StartPathHandlerCommand(pathHandler),
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new PathHandlerShootCommand(
                pathHandler,
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem),
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new PathHandlerShootCommand(
                pathHandler,
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem)
            // new ToggleVisionUpdatesCommand(driveSubsystem)
            ));
  }

  public void generateTrajectory() {
    pathHandler.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
