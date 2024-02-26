package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class ShooterConstants {
  public static final int kLeftShooterTalonID = 40;
  public static final int kRightShooterTalonID = 41;
  public static final double kCloseEnough = 0.2;
  public static final double kShootTime = 1.0;
  public static final double kPodiumSpeed = 0;

  public static final TalonFXConfiguration getShooterConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.5;
    slot0.kI = 0.01;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.110;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs().withMotionMagicAcceleration(130).withMotionMagicJerk(1000);
    config.MotionMagic = motionMagic;

    return config;
  }

  public static final CurrentLimitsConfigs getShooterSupplyLimitConfig() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 0.0;
    config.StatorCurrentLimitEnable = false;

    config.SupplyCurrentLimit = 40;
    config.SupplyCurrentThreshold = 40;
    config.SupplyTimeThreshold = 0.5;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }
}
