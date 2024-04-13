package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetHolonomicControllerCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private double yaw;

  public ResetHolonomicControllerCommand(DriveSubsystem driveSubsystem, double yaw) {
    this.driveSubsystem = driveSubsystem;
    this.yaw = yaw;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetHolonomicController(yaw);
  }
}
