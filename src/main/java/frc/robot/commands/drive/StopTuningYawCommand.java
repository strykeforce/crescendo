package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class StopTuningYawCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;

  public StopTuningYawCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setIsTuningYaw(false);
  }
}
