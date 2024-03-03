package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveAutonWithHeadingCommand;
import frc.robot.commands.robotState.DistanceShootCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class NonAmpAutoCommand extends SequentialCommandGroup {
  private DriveAutonWithHeadingCommand initialToNote5;
  private DriveAutonCommand note5ToShoot;
  private DriveAutonCommand shootToNote4;
  private DriveAutonCommand note4ToShoot;
  private boolean hasGenerated = false;

  public NonAmpAutoCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    initialToNote5 =
        new DriveAutonWithHeadingCommand(
            driveSubsystem,
            "NonAmpInitial1_MiddleNote5",
            true,
            true,
            Rotation2d.fromDegrees(-33.2),
            0.0,
            true);
    note5ToShoot = new DriveAutonCommand(driveSubsystem, "MiddleNote5_NonAmpShoot1", true, false);
    shootToNote4 = new DriveAutonCommand(driveSubsystem, "NonAmpShoot1_MiddleNote4", true, false);
    note4ToShoot = new DriveAutonCommand(driveSubsystem, "MiddleNote4_NonAmpShoot1", true, false);

    addCommands(
        new SequentialCommandGroup(
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            new DistanceShootCommand(
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem,
                AutonConstants.kNAI1ToSpeakerDist),
            initialToNote5,
            note5ToShoot,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
            shootToNote4,
            note4ToShoot,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)
            // new ToggleVisionUpdatesCommand(driveSubsystem)
            ));
  }

  public void generateTrajectory() {
    initialToNote5.generateTrajectory();
    note5ToShoot.generateTrajectory();
    shootToNote4.generateTrajectory();
    note4ToShoot.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}