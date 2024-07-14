package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import net.jafama.FastMath;

public class TurnUntilAngleCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double target;
  private double vOmega;

  public TurnUntilAngleCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      Rotation2d target,
      double vOmega) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.target = target.getDegrees();
    this.vOmega = vOmega;

    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      this.target = Units.radiansToDegrees(FastMath.normalizeZeroTwoPi(target.getRadians()));
    }
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
    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) {
      return driveSubsystem.getGyroRotation2d().getDegrees() <= target;
    } else {
      return driveSubsystem.getGyroRotation2d().getDegrees() >= target;
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
