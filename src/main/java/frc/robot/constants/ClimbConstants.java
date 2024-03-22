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
  public static final double kZeroPct = -0.5;
  public static final double kZeroStableCounts = 5;
  public static final double kZeroSpeedThreshold = 1;

  // Forks
  public static final double kZeroForkPct = -0.35;
  public static final double kZeroForkMaxVel = 0.5;
  public static final int kForkZeroStableCounts = 5;
  public static final double kCloseEnoughForks = 25; // 100
  public static final double kLeftExtendPos = 400; // 500
  public static final double kRightExtendPos = 400; // 500
  public static final double kLeftRetractPos = 20; // 10
  public static final double kRightRetractPos = 20; // 10
  public static final double kRevsPerInch = 1.788; // 1.192 -> 2.98

  // PRE-CLIMB
  public static final double kLeftClimbPrepPos = 80.0; // 70
  public static final double kRightClimbPrepPos = 80.0;

  public static final double kLeftClimbHighPrepPos = 93.576;
  public static final double kRightClimbHighPrepPos = 93.576;

  // TRAP CLIMB
  public static final double kLeftClimbTrapPos =
      10.268; // 12.056 <- 13.844 <- 17.42 <- 11.46 <- 5.5
  public static final double kRightClimbTrapPos = 10.268; // 5.5

  public static final double kLeftStowPos = 2.5;
  public static final double kRightStowPos = 2.5;

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
    motionMagic.MotionMagicAcceleration = 60; // 80
    motionMagic.MotionMagicCruiseVelocity = 350; // 350
    motionMagic.MotionMagicJerk = 1500; // 3000
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
    motionMagic.MotionMagicAcceleration = 60; // 80
    motionMagic.MotionMagicCruiseVelocity = 350; // 700
    motionMagic.MotionMagicJerk = 1500; // 3000
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

    config.forwardSoftLimitThreshold = 510;
    config.forwardSoftLimitEnable = true; // fixme

    config.reverseSoftLimitThreshold = 0.0;
    config.reverseSoftLimitEnable = true; // fixme

    config.continuousCurrentLimit = 2;
    config.peakCurrentLimit = 2;
    config.peakCurrentDuration = 100;

    config.slot0.kP = 20.0;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.0;
    config.slot0.kF = 12.0;
    config.slot0.integralZone = 0;
    config.slot0.maxIntegralAccumulator = 0;
    config.slot0.allowableClosedloopError = 0;
    config.motionCruiseVelocity = 75;
    config.motionAcceleration = 500;
    config.neutralDeadband = 0.01;
    config.velocityMeasurementWindow = 64;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;

    return config;
  }
}
