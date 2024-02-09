package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class WristConstants {
  public static final int kWristTalonSrxId = 35;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  public static final double kWristZeroTicks = RobotConstants.kWristZero;
  public static final int kMinBeamBreaks = 0;

  public static final double testWristPos = 0.0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.forwardSoftLimitThreshold = 610;
    config.forwardSoftLimitEnable = true;

    config.reverseSoftLimitThreshold = -3089.0;
    config.reverseSoftLimitEnable = true;

    config.continuousCurrentLimit = 10;
    config.peakCurrentDuration = 40;
    config.peakCurrentLimit = 15;

    config.slot0.kP = 2.0;
    config.slot0.kI = 0.0;
    config.slot0.kD = 20.0;
    config.slot0.kF = 2.0;
    config.slot0.integralZone = 0;
    config.slot0.maxIntegralAccumulator = 0;
    config.slot0.allowableClosedloopError = 0;
    config.motionCruiseVelocity = 400;
    config.motionAcceleration = 700;
    config.neutralDeadband = 0.01;
    config.velocityMeasurementWindow = 64;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;

    return config;
  }
}
