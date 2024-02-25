package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ClimbConstants {
  public static final int kLeftClimbFxId = 50;
  public static final int kRightClimbFxId = 51;
  public static final int kLeftRatchetId = 0;
  public static final int kRightRatchetId = 1;
  public static final int kLeftTrapBarId = 2;
  public static final int kRightTrapBarId = 3;
  public static final int kLeftForkSRXId = 52;
  public static final int kRightForkSRXId = 53;

  public static final double kCloseEnoughRots = 0.5;
  public static final double kMaxRots = 100.0;
  public static final double kMinRots = 0.0;
  public static final double kZeroPct = -0.1;
  public static final double kZeroStableCounts = 3;
  public static final double kZeroSpeedThreshold = 1;

  public static final double kLeftForkZero = 0.0;
  public static final double kRightForkZero = 0.0;
  public static final double kCloseEnoughForks = 100;
  public static final double kLeftExtendPos = 100;
  public static final double kRightExtendPos = 100;
  public static final double kLeftRetractPos = 0.0;
  public static final double kRightRetractPos = 0.0;

  public static TalonFXConfiguration getLeftConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits = getRunCurrentLimit();

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxRots;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinRots;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 3.0;
    slot0.kI = 4.0;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.120;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 80;
    motionMagic.MotionMagicCruiseVelocity = 700;
    motionMagic.MotionMagicJerk = 3000;
    config.MotionMagic = motionMagic;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorOutput;

    return config;
  }

  public static TalonFXConfiguration getRightConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits = getRunCurrentLimit();

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxRots;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinRots;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 3.0;
    slot0.kI = 4.0;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kV = 0.120;
    slot0.kA = 0.0;
    slot0.kG = 0.0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0 = slot0;

    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 80;
    motionMagic.MotionMagicCruiseVelocity = 700;
    motionMagic.MotionMagicJerk = 3000;
    config.MotionMagic = motionMagic;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorOutput;

    return config;
  }

  public static CurrentLimitsConfigs getZeroCurrentLimit() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 20.0;
    config.StatorCurrentLimitEnable = true;

    config.SupplyCurrentLimit = 5;
    config.SupplyCurrentThreshold = 5;
    config.SupplyTimeThreshold = 0.1;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static CurrentLimitsConfigs getRunCurrentLimit() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    config.StatorCurrentLimit = 0.0;
    config.StatorCurrentLimitEnable = false;

    config.SupplyCurrentLimit = 25;
    config.SupplyCurrentThreshold = 25;
    config.SupplyTimeThreshold = 0.5;
    config.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static TalonSRXConfiguration getForkConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.forwardSoftLimitThreshold = 0;
    config.forwardSoftLimitEnable = false; // fixme

    config.reverseSoftLimitThreshold = 0.0;
    config.reverseSoftLimitEnable = false; // fixme

    config.continuousCurrentLimit = 10;
    config.peakCurrentLimit = 15;
    config.peakCurrentDuration = 40;

    config.slot0.kP = 0.0;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.0;
    config.slot0.kF = 0.0;
    config.slot0.integralZone = 0;
    config.slot0.maxIntegralAccumulator = 0;
    config.slot0.allowableClosedloopError = 0;
    config.motionCruiseVelocity = 0;
    config.motionAcceleration = 0;
    config.neutralDeadband = 0.01;
    config.velocityMeasurementWindow = 64;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;

    return config;
  }
}
