package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public final class WristConstants {
  public static final int kWristTalonSrxId = 35;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  // public static final double kWristZeroTicks = 0;
  public static final int kMinBeamBreaks = 0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.continuousCurrentLimit = 20;
    config.peakCurrentDuration = 20;
    config.peakCurrentLimit = 25;

    return config;
  }
}
