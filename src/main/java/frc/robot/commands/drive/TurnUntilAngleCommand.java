package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import net.jafama.FastMath;

public class TurnUntilAngleCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double targetRads;
  private double vOmega;
  private boolean normalizedZeroTwoPi = false;

  public TurnUntilAngleCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      double target,
      double vOmega) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.targetRads = target;
    this.vOmega = vOmega;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      this.vOmega = -vOmega;
      this.targetRads = FastMath.normalizeMinusPiPi(FastMath.PI - targetRads);
      driveSubsystem.setAutoDebugMsg(
          "Flipping for RED: vOmega = " + vOmega + ", targetRads = " + targetRads);
    }

    double currRads = driveSubsystem.getGyroRotation2d().getRadians();

    if (vOmega > 0 && targetRads < currRads || vOmega < 0 && targetRads > currRads) {
      targetRads = FastMath.normalizeZeroTwoPi(targetRads);
      normalizedZeroTwoPi = true;
      driveSubsystem.setAutoDebugMsg("Normalizing 0 - 2 pi: targetRads = " + targetRads);
    }

    driveSubsystem.setAutoDebugMsg("Turning until " + FastMath.toDegrees(targetRads));
    driveSubsystem.move(0, 0, vOmega, false);
  }

  @Override
  public void execute() {
    driveSubsystem.move(0, 0, vOmega, false);
  }

  @Override
  public boolean isFinished() {
    double currRads = driveSubsystem.getGyroRotation2d().getRadians();

    if (normalizedZeroTwoPi) {
      currRads = FastMath.normalizeZeroTwoPi(currRads);
    }

    // logger.info("vOmega = {}, currRads = {}, targetRads = {}", vOmega, currRads, targetRads);

    return vOmega > 0 && currRads >= targetRads || vOmega < 0 && currRads <= targetRads;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setAutoDebugMsg("Finished turning until " + FastMath.toDegrees(targetRads));
  }
}
