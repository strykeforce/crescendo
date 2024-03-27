package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import java.util.function.DoubleSupplier;
import org.strykeforce.thirdcoast.util.ExpoScale;

public class TuneYawCommand extends Command {
  private DoubleSupplier fwdStick;
  private DoubleSupplier strStick;
  private DoubleSupplier yawStick;
  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private double[] rawValues = new double[3];
  private final ExpoScale expoScaleYaw =
      new ExpoScale(DriveConstants.kDeadbandAllStick, DriveConstants.kExpoScaleYawFactor);
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter strLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter yawLimiter = new SlewRateLimiter(DriveConstants.kRateLimitYaw);

  public TuneYawCommand(
      DoubleSupplier fwdStick,
      DoubleSupplier strStick,
      DoubleSupplier yawStick,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.fwdStick = fwdStick;
    this.strStick = strStick;
    this.yawStick = yawStick;
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.setIsTuningYaw(true);
  }

  @Override
  public void execute() {
    rawValues[0] = fwdStick.getAsDouble();
    rawValues[1] = strStick.getAsDouble();
    rawValues[2] = yawStick.getAsDouble();

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) {
      driveSubsystem.drive(
          -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(fwdStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(strStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      yawStick.getAsDouble(), DriveConstants.kDeadbandAllStick)));

    } else {
      driveSubsystem.drive(
          DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(fwdStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(strStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          -DriveConstants.kMaxAccelOmegaPath
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      yawStick.getAsDouble(), DriveConstants.kDeadbandAllStick)));
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(
                driveSubsystem.getPoseMeters().getRotation().getDegrees()
                    - DriveConstants.kYawTuningTarget)
            <= DriveConstants.kDegreesCloseEnough
        || !driveSubsystem.getIsTuningYaw();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setIsTuningYaw(false);
  }
}
