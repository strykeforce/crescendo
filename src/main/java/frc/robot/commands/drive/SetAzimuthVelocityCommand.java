package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetAzimuthVelocityCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private double vel;

  public SetAzimuthVelocityCommand(DriveSubsystem driveSubsystem, double vel) {
    this.driveSubsystem = driveSubsystem;
    this.vel = vel;
  }

  @Override
  public void initialize() {
    driveSubsystem.setAzimuthVel(vel);
  }
}
