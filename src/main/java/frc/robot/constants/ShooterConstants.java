package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public final class ShooterConstants {
  public static final int kLeftShooterTalonID = 40;
  public static final int kRightShooterTalonID = 41;
  public static final double kCloseEnough = 3.0; // 0.75
  public static final double kShootTime = 0.04; // 0.2 - now delay after beam unbroken
  public static final double kPodiumShootTime = 0.5;
  public static final double kPodiumSpeed = 0;

  public static final TalonFXConfiguration getLeftShooterConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.4;
    slot0.kI = 0.1;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.120;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs().withMotionMagicAcceleration(130).withMotionMagicJerk(1000);
    config.MotionMagic = motionMagic;

    config.CurrentLimits = getShooterSupplyLimitConfig();

    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    motorConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = motorConfig;

    return config;
  }

  public static final TalonFXConfiguration getRightShooterConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.4;
    slot0.kI = 0.1;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.120;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs().withMotionMagicAcceleration(130).withMotionMagicJerk(1000);
    config.MotionMagic = motionMagic;

    config.CurrentLimits = getShooterSupplyLimitConfig();

    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    motorConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = motorConfig;

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

  public static final TalonFXConfiguration getSideBySideShooterConfig() {
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
}
