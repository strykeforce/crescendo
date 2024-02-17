package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class NonAmpAutoCommand extends SequentialCommandGroup {
  private DriveAutonCommand initialToNote5;
  private DriveAutonCommand note5ToShoot;
  private DriveAutonCommand shootToNote4;
  private boolean hasGenerated = false;

  public NonAmpAutoCommand(DriveSubsystem driveSubsystem) {
    initialToNote5 =
        new DriveAutonCommand(driveSubsystem, "NonAmpInitial1_MiddleNote5", true, true);
    note5ToShoot = new DriveAutonCommand(driveSubsystem, "MiddleNote5_NonAmpShoot1", true, false);
    shootToNote4 = new DriveAutonCommand(driveSubsystem, "NonAmpShoot1_MiddleNote4", true, false);

    addCommands(
        new SequentialCommandGroup(
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-60)),
            initialToNote5, note5ToShoot, shootToNote4
            // new ToggleVisionUpdatesCommand(driveSubsystem)
            ));
  }

  public void generateTrajectory() {
    initialToNote5.generateTrajectory();
    note5ToShoot.generateTrajectory();
    shootToNote4.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
