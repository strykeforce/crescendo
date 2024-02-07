package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class MagazineConstants {
  public static final int kMagazineFalconID = 25;
  public static final double kCloseEnough = 300;
  public static final double kShootCloseEnough = 500;
  // public static final double kFeedingSpeed = 0.5;
  public static final double kShootSpeed = 0;
  public static final int kMinBeamBreaks = 3;
  public static final double kIntakingSpeed = -0.5;
  public static final double kEmptyingSpeed = -0.75;
  public static final double kReversingSpeed = 0.05; // TODO do testing to determine correct speed

  public static final double kPodiumPrepareSpeed = 0.1;

  public static final TalonFXConfiguration getMagazineConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

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

  public static final CurrentLimitsConfigs getMagazineSupplyLimitConfig() {
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
