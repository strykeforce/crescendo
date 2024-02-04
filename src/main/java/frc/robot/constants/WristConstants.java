package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public final class WristConstants {
  public static final int kWristTalonSrxId = 35;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  public static final double kWristZeroTicks = RobotConstants.kWristZero;
  public static final int kMinBeamBreaks = 0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.forwardSoftLimitThreshold = 610;
    config.forwardSoftLimitEnable = true;

    config.reverseSoftLimitThreshold = -3089.0;
    config.reverseSoftLimitEnable = true;

    config.continuousCurrentLimit = 20;
    config.peakCurrentDuration = 20;
    config.peakCurrentLimit = 25;

    return config;
  }
}
