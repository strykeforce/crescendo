package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class ExampleConstants {
  public static int kExampleSrxId = 5;
  public static int kExampleFxId = 6;

  public static final double kCloseEnough = 100;
  public static final double kMaxFwd = 100;
  public static final double kMaxRev = -100;
  public static final double kZeroTicks = 1530;

  // Example Talon SRX Config
  public static TalonSRXConfiguration getSRXConfig() {
    TalonSRXConfiguration srxConfig = new TalonSRXConfiguration();

    // General Configs
    srxConfig.neutralDeadband = 0.04;
    srxConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    srxConfig.velocityMeasurementWindow = 64;
    srxConfig.voltageCompSaturation = 12;
    srxConfig.voltageMeasurementFilter = 32;

    // PID Configs
    srxConfig.slot0.kP = 0.0;
    srxConfig.slot0.kI = 0.0;
    srxConfig.slot0.kD = 0.0;
    srxConfig.slot0.kF = 0.0;
    srxConfig.slot0.allowableClosedloopError = 0;
    srxConfig.slot0.integralZone = 0.0;
    srxConfig.slot0.maxIntegralAccumulator = 0.0;
    srxConfig.motionCruiseVelocity = 0.0;
    srxConfig.motionAcceleration = 0.0;

    // Soft Limits
    srxConfig.forwardSoftLimitEnable = true;
    srxConfig.forwardSoftLimitThreshold = kMaxFwd;
    srxConfig.reverseSoftLimitEnable = true;
    srxConfig.reverseSoftLimitThreshold = kMaxRev;

    // Hard Limits
    srxConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    srxConfig.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    srxConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
    srxConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;

    // Current Limit
    srxConfig.continuousCurrentLimit = 10; // A
    srxConfig.peakCurrentLimit = 15; // A
    srxConfig.peakCurrentDuration = 100; // ms

    return srxConfig;
  }

  // Example Talon FX Config
  public static TalonFXConfiguration getFXConfig() {
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(10)
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentThreshold(25)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyTimeThreshold(0.02);
    fxConfig.CurrentLimits = current;

    HardwareLimitSwitchConfigs hwLimit =
        new HardwareLimitSwitchConfigs()
            .withForwardLimitAutosetPositionEnable(false)
            .withForwardLimitEnable(false)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
            .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
            .withReverseLimitAutosetPositionEnable(false)
            .withReverseLimitEnable(false)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin);
    fxConfig.HardwareLimitSwitch = hwLimit;

    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kMaxFwd)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kMaxRev);
    fxConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0);
    fxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);
    fxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Coast);
    fxConfig.MotorOutput = motorOut;

    return fxConfig;
  }
}
