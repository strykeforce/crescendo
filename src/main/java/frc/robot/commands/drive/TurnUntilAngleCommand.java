package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnUntilAngleCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private Rotation2d target;
  private double vOmega;

  public TurnUntilAngleCommand(DriveSubsystem driveSubsystem, Rotation2d target, double vOmega) {
    this.driveSubsystem = driveSubsystem;
    this.target = target;
    this.vOmega = vOmega;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.move(0, 0, vOmega, false);
  }

  @Override
  public void execute() {
    driveSubsystem.move(0, 0, vOmega, false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getGyroRotation2d().getDegrees() - target.getDegrees())
        < DriveConstants.kDegreesCloseEnough;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0, 0, vOmega, false);
  }
}
