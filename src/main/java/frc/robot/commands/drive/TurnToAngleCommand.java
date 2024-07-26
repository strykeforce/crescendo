package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import net.jafama.FastMath;

public class TurnToAngleCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;
  Rotation2d angle;

  public TurnToAngleCommand(DriveSubsystem driveSubsystem, Rotation2d angle) {
    this.driveSubsystem = driveSubsystem;
    this.angle = angle;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOmegaController();
    driveSubsystem.move(0, 0, driveSubsystem.getvOmegaToTarget(angle), true);
  }

  @Override
  public void execute() {
    driveSubsystem.move(0, 0, driveSubsystem.getvOmegaToTarget(angle), true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getGyroRotation2d().getDegrees() - angle.getDegrees())
            < DriveConstants.kDegreesCloseEnough
        || Math.abs(
                Units.radiansToDegrees(
                    FastMath.normalizeZeroTwoPi(driveSubsystem.getGyroRotation2d().getRadians())
                        - FastMath.normalizeZeroTwoPi(angle.getRadians())))
            < DriveConstants.kDegreesCloseEnough;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0, 0, 0, true);
  }
}
