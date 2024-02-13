package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class XLockCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;

  public XLockCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.xLock();
  }
}
