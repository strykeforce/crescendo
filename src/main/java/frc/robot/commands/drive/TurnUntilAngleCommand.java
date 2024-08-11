package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import net.jafama.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurnUntilAngleCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double targetRads;
  private double vOmega;
  private double prevRads;
  private boolean containsDiscontinuity = false;
  private static final Logger logger = LoggerFactory.getLogger(TurnUntilAngleCommand.class);

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
    logger.info("targetRads initial: {}", targetRads);
    targetRads = FastMath.normalizeZeroTwoPi(targetRads);

    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      this.vOmega = -vOmega;
      this.targetRads = FastMath.normalizeZeroTwoPi(FastMath.PI - targetRads);
      logger.info("Flipping for RED: vOmega = {} targetRads = {}", vOmega, targetRads);
    }

    double currRads = FastMath.normalizeZeroTwoPi(driveSubsystem.getGyroRotation2d().getRadians());

    logger.info("vOmega = {} currRads = {} targetRads = {}", vOmega, currRads, targetRads);

    prevRads = currRads;

    if (vOmega > 0 && targetRads < currRads || vOmega < 0 && targetRads > currRads) {
      targetRads = FastMath.normalizeZeroTwoPi(targetRads);
      containsDiscontinuity = true;
      logger.info("Yaw must cross 0: currRads = {}, targetRads = {}", currRads, targetRads);
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
    double currRads = FastMath.normalizeZeroTwoPi(driveSubsystem.getGyroRotation2d().getRadians());

    if (containsDiscontinuity) {
      if (FastMath.abs(currRads - prevRads) > FastMath.PI) {
        containsDiscontinuity = false;
        logger.info("Crossed discontinuity");
      }
    }

    prevRads = currRads;

    return !containsDiscontinuity
        && (vOmega > 0 && currRads >= targetRads || vOmega < 0 && currRads <= targetRads);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setAutoDebugMsg("Finished turning until " + FastMath.toDegrees(targetRads));
  }
}
