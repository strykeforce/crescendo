package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class ShooterConstants {
  public static final int kLeftShooterTalonID = 40;
  public static final int kRightShooterTalonID = 41;
  public static final double kCloseEnough = 100;
  public static final double kShootTime = 1.0;
  public static final double kPodiumSpeed = 0;

  public static final TalonFXConfiguration getShooterConfig() {
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

  public static final CurrentLimitsConfigs getShooterSupplyLimitConfig() {
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
