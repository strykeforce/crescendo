package frc.robot.commands.drive;

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
      double target,
      double vOmega) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.target = target;
    this.vOmega = vOmega;

    addRequirements(driveSubsystem);
  }

  // Must call in generateTrajectory()
  public void updateColor() {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      this.vOmega = -vOmega;
      this.target =
          Units.radiansToDegrees(FastMath.normalizeZeroTwoPi(Units.degreesToRadians(180 - target)));
    }
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
      return Units.radiansToDegrees(
              FastMath.normalizeZeroTwoPi(driveSubsystem.getGyroRotation2d().getRadians()))
          >= target;
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
