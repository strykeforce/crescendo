package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class NonAmpInitial_Note3Command extends SequentialCommandGroup {
  private DriveAutonCommand initialToNote3;
  private DriveAutonCommand note3ToShoot;
  private boolean hasGenerated = false;

  public NonAmpInitial_Note3Command(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    initialToNote3 =
        new DriveAutonCommand(driveSubsystem, "NonAmpInitial1_MiddleNote3", true, true);
    note3ToShoot = new DriveAutonCommand(driveSubsystem, "MiddleNote3_NonAmpShoot2", true, false);

    addCommands(
        new SequentialCommandGroup(
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
            initialToNote3,
            note3ToShoot,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)));
  }

  public void generateTrajectory() {
    initialToNote3.generateTrajectory();
    note3ToShoot.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
