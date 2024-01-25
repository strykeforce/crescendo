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

public class DriveTeleopCommand extends Command {
  private DoubleSupplier fwdStick;
  private DoubleSupplier strStick;
  private DoubleSupplier yawStick;
  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private double[] rawValues = new double[3];
  private final ExpoScale expoScaleYaw =
      new ExpoScale(DriveConstants.kDeadbandAllStick, DriveConstants.kExpoScaleYawFactor);
  // private final RateLimit rateLimitYaw = new RateLimit(DriveConstants.kRateLimitYaw);
  // private final RateLimit rateLimitMove = new RateLimit(DriveConstants.kRateLimitMove);
  // private double[] adjustedValues = new double[3];
  // private final double vectorOffset =
  //         / (DriveConstants.kExpoScaleMoveFactor
  //                 * Math.pow((Math.sqrt(2) - DriveConstants.kDeadbandAllStick), 3)
  //             + (1 - DriveConstants.kExpoScaleMoveFactor)
  //                 * (Math.sqrt(2) - DriveConstants.kDeadbandAllStick));
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter strLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter yawLimiter = new SlewRateLimiter(DriveConstants.kRateLimitYaw);

  public DriveTeleopCommand(
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
    } else if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.drive(
          -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(
                      -fwdStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(
                      -strStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      yawStick.getAsDouble(), DriveConstants.kDeadbandAllStick)));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}
