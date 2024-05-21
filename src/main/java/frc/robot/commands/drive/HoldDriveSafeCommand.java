package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class HoldDriveSafeCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;

  public HoldDriveSafeCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.toSafeHold();
  }
}
