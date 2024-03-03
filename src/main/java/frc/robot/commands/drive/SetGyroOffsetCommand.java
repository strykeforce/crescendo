package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetGyroOffsetCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private Rotation2d offset;

  public SetGyroOffsetCommand(DriveSubsystem driveSubsystem, Rotation2d offset) {
    this.driveSubsystem = driveSubsystem;
    this.offset = offset;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setGyroOffset(offset);
  }
}
