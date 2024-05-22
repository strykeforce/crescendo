package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public final class MagazineConstants {
  public static final int kMagazineFalconID = 25;
  public static final double kCloseEnough = 10;
  public static final double kShootCloseEnough = 10;
  // public static final double kFeedingSpeed = 0.5;
  public static final int kMinBeamBreaks = 0; // 3
  public static final double kIntakingSpeed = -50; // -46
  public static final double kIdleIntakingSpeed = -25;
  public static final double kFastIntakingSpeed = -50; // -90
  public static final double kEmptyingSpeed = -50; // -90 -> -70
  public static final double kEjectingSpeed = 50;
  public static final double kReversingSpeed = 9.6; // 4.8

  public static final double kReleaseTime = 0.75;
  public static final double kPodiumPrepareSpeed = -20; // -10
  public static final double kPodiumShootSpeed = 105; // 90
  public static final double kTrapReleaseSpeed = -40.0; // -20.0
  public static final double kTrapReleaseTime = 0.75;
  public static final double kAmpReleaseSpeed = 40.0; // 20.0
  public static final double kPodiumRumbleSpeed = 80;

  public static final TalonFXConfiguration getMagazineConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.2;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.125;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs().withMotionMagicAcceleration(500).withMotionMagicJerk(5000);
    config.MotionMagic = motionMagic;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    motorConfig.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorConfig;

    return config;
  }

  public static final CurrentLimitsConfigs getMagazineSupplyLimitConfig() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 0.0;
    config.StatorCurrentLimitEnable = false;

    config.SupplyCurrentLimit = 40;
    config.SupplyCurrentThreshold = 50;
    config.SupplyTimeThreshold = 1;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }
}
