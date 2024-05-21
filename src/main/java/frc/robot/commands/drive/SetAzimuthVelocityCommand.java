package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetAzimuthVelocityCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private double vel;

  public SetAzimuthVelocityCommand(DriveSubsystem driveSubsystem, double vel) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.vel = vel;
  }

  @Override
  public void initialize() {
    driveSubsystem.setAzimuthVel(vel);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
