package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class ElbowConstants {
  public static final int kElbowTalonFxId = 30;
  public static final int kRemoteEncoderID = 31;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;

  public static final double kAbsEncoderToMechRatio = 2;
  public static final double kFxGearbox = 50;
  public static final double kFxPulley = 2;
  public static final double kFxChain = 50.0 / 24.0;
  public static final double kElbowZeroRots = 0.11206;

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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 24.44;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -45.15;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);
    config.MotionMagic = motionMagic;

    return config;
  }

  public static CurrentLimitsConfigs getCurrentLimitConfig() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 0.0;
    config.StatorCurrentLimitEnable = false;

    config.SupplyCurrentLimit = 20;
    config.SupplyCurrentThreshold = 25;
    config.SupplyTimeThreshold = .02;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }
}
