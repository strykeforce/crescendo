package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class WristConstants {
  public static final int kWristTalonSrxId = 35;
  public static final double kCloseEnoughTicks = 50;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  public static final double kWristZeroTicks = RobotConstants.kWristZero;
  public static final int kMinBeamBreaks = 0;

  public static final double testWristPos = 0.0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.forwardSoftLimitThreshold = 1515;
    config.forwardSoftLimitEnable = true; // fixme

    config.reverseSoftLimitThreshold = -2200.0;
    config.reverseSoftLimitEnable = true; // fixme

    config.continuousCurrentLimit = 10;
    config.peakCurrentLimit = 15;
    config.peakCurrentDuration = 40;

    config.slot0.kP = 1.5;
    config.slot0.kI = 0.0;
    config.slot0.kD = 10.0;
    config.slot0.kF = 0.95;
    config.slot0.integralZone = 0;
    config.slot0.maxIntegralAccumulator = 0;
    config.slot0.allowableClosedloopError = 0;
    config.motionCruiseVelocity = 600;
    config.motionAcceleration = 2000;
    config.neutralDeadband = 0.01;
    config.velocityMeasurementWindow = 64;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;

    return config;
  }
}
