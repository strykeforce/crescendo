package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class ElbowConstants {
  public static final int kElbowTalonFxId = 30;
  public static final int kRemoteEncoderID = 31;
  public static final double kCloseEnoughRots = 1;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;

  public static final double kAbsEncoderToMechRatio = 68.0 / 38.0;
  public static final double kFxGearbox = 50;
  public static final double kFxPulley = 2;
  public static final double kFxChain = 50.0 / 24.0;

  public static final double kElbowTestPos = 0.0;
  public static final double kZeroPos = 25.0;
  public static final double kZeroOffset = 0.00276; // 1 degree = 0.00276

  // Zero Recovery Constants
  public static final double kZeroVelocity = 0.05;
  public static final double kMinVelocityZeroing = 1;
  public static final int kMinStableZeroCounts = 5;
  public static final int kStableCountsAbsEncoder = 3;
  public static final double kCloseEnoughAbs = 0.0001;

  public static CANcoderConfiguration getCanCoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    return config;
  }

  public static TalonFXConfiguration getFxConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits = getCurrentLimitConfig();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 34;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -100;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 2.0; // 1.2
    slot0.kI = 2.0;
    slot0.kD = 0.2;
    slot0.kS = 0.0;
    slot0.kV = 0.110;
    slot0.kA = 0.0;
    slot0.kG = -0.150;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(400)
            .withMotionMagicCruiseVelocity(90)
            .withMotionMagicJerk(5000);
    config.MotionMagic = motionMagic;

    return config;
  }

  public static CurrentLimitsConfigs getCurrentLimitConfig() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 0.0;
    config.StatorCurrentLimitEnable = false;

    config.SupplyCurrentLimit = 20;
    config.SupplyCurrentThreshold = 20;
    config.SupplyTimeThreshold = .02;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static CurrentLimitsConfigs getZeroCurrentLimitConfig() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 10.0;
    config.StatorCurrentLimitEnable = true;

    config.SupplyCurrentLimit = 5;
    config.SupplyCurrentThreshold = 5;
    config.SupplyTimeThreshold = 0.1;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }
}
