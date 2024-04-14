package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetOmegaContKPCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private double kP;
  private double accel;

  public SetOmegaContKPCommand(DriveSubsystem driveSubsystem, double kP, double accel) {
    this.driveSubsystem = driveSubsystem;
    this.kP = kP;
    this.accel = accel;
  }

  @Override
  public void initialize() {
    driveSubsystem.setOmegaKP(kP, accel);
  }
}
