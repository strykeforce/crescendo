package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ToggleVisionUpdatesCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;

  public ToggleVisionUpdatesCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.enableVisionUpdates(!driveSubsystem.usingVisionUpdates());
  }
}
