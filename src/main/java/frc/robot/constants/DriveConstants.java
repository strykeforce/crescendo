package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
// import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.ArrayList;

public final class DriveConstants {
  public static final double kDeadbandAllStick = 0.075;
  public static final double kExpoScaleYawFactor = 0.75;
  public static final double kRateLimitFwdStr = 3.5;
  public static final double kRateLimitYaw = 8.0;

  public static final double kFieldMaxX = 16.540988; // m
  public static final double kFieldMaxY = 8.21055; // m

  public static final int kTalonConfigTimeout = 10; // ms

  public static final int kPigeonCanID = 4;

  public static final double kRobotLength = 0.5461;
  public static final double kRobotWidth = 0.6922;

  public static final double kBumperRobotLength = 0.84;
  public static final double kBumperRobotWidth = 1.0;

  public static final double kSpeedStillFeedThreshold = 0.5;
  public static final double kSpeedStillThreshold = 0.1; // meters per second
  public static final double kGyroRateStillThreshold = 10.0; // 25  5 degrees per second
  public static final double kDegreesCloseEnough = 3;
  public static final double kDegreesCloseEnoughFeeding = 5;

  // Move and shoot thresholds
  public static final double kMaxStableAccel = 0.1;
  public static final int kVelocityStableCounts = 5;
  public static final double kMaxMoveShootVelocity = 2.0;
  public static final double kMaxMoveGyroRateThreshold = 25.0; // 10
  public static final double kMoveShootVelDetune = 0.2;
  public static final double kMoveShootTeleMaxVelX = 2.0;
  public static final double kMaxSpeakerDist = 8.0;

  public static final double kDriveMotorOutputGear = 34; // 30
  public static final double kDriveInputGear = 42;
  public static final double kBevelInputGear = 15;
  public static final double kBevelOutputGear = 45;

  public static final double kDriveGearRatio =
      (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);
  //   public static final double kWheelDiameterInches = 3.0 * 506.0 / 500.0;
  public static final double kMaxSpeedMetersPerSecond = 6.495;

  public static final double kMaxOmega =
      (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
          / 2.0; // wheel locations below

  public static final double kPlaceMovePercent = 0.2;
  public static final double kPlaceYawPercent = 0.2;
  public static final double kIntakeYawPercent = .75;
  public static final double kIntakeMovePercent = .75;

  public static final int kTempAvgCount = 25;
  public static final double kTripTemp = 1300;
  public static final double kRecoverTemp = 1290;
  public static final double kNotifyTemp = 1295;

  public static final double supplyCurrentLimitNorm = 50.0;
  public static final double supplyCurrentThreshold = 55.0;

  public static Translation2d[] getWheelLocationMeters() {
    final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
    final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
    Translation2d[] locs = new Translation2d[4];
    locs[0] = new Translation2d(x, y); // left front
    locs[1] = new Translation2d(x, -y); // right front
    locs[2] = new Translation2d(-x, y); // left rear
    locs[3] = new Translation2d(-x, -y); // right rear
    return locs;
  }

  public static TalonSRXConfiguration getAzimuthTalonConfig() {
    // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

    azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

    azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;

    azimuthConfig.slot0.kP = 15.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 150.0;
    azimuthConfig.slot0.kF = 1.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.slot0.maxIntegralAccumulator = 0;

    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    azimuthConfig.voltageCompSaturation = 12;
    azimuthConfig.voltageMeasurementFilter = 32;
    azimuthConfig.neutralDeadband = 0.04;
    return azimuthConfig;
  }

  // Drive Falcon Config
  public static TalonFXConfiguration getDriveTalonConfig() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 45; // 40
    currentConfig.SupplyCurrentThreshold = 50; // 45
    currentConfig.SupplyTimeThreshold = 0.0;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    driveConfig.CurrentLimits = currentConfig;

    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = 0.5; // 0.16 using phoenix 6 migrate
    slot0Config.kI = 0.5; // 0.0002 using phoenix 6 migrate
    slot0Config.kD = 0.0;
    slot0Config.kV = 0.12; // 0.047 using phoenix 6 migrate
    driveConfig.Slot0 = slot0Config;

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.DutyCycleNeutralDeadband = 0.01;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput = motorConfigs;

    return driveConfig;
  }

  public static CurrentLimitsConfigs getSafeDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 30; // 40
    currentConfig.SupplyCurrentThreshold = 35; // 45
    currentConfig.SupplyTimeThreshold = 0.0;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    return currentConfig;
  }

  public static CurrentLimitsConfigs getNormDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 45; // 40
    currentConfig.SupplyCurrentThreshold = 50; // 45
    currentConfig.SupplyTimeThreshold = 0.0;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    return currentConfig;
  }

  public static Pigeon2Configuration getPigeon2Configuration() {
    Pigeon2Configuration config = new Pigeon2Configuration();

    config.MountPose.MountPoseYaw = -90.0;
    config.MountPose.MountPoseRoll = 0.0;
    config.MountPose.MountPosePitch = 0.0;

    config.GyroTrim.GyroScalarX = 0.0;
    config.GyroTrim.GyroScalarY = 0.0;
    config.GyroTrim.GyroScalarZ = -2.12;

    return config;
  }

  // Holonomic Controller Constants
  public static final double kXPHolonomic = 3.0; // was 3
  public static final double kYPHolonomic = 3.0; // was 3
  public static final double kIHolonomic = 0.0000;
  public static final double kDHolonomic = 0.00; // kPHolonomic/100
  public static final double kIMin = 0.0;
  public static final double kIMax = 0.0;

  public static final double kPOmegaSpin = 9.0; // 4.5
  public static final double kPOmega = 4.5; // 4.5
  public static final double kIOmega = 0.0;
  public static final double kDOmega = 0.0; // 0.001
  //    public static final double kMaxVelOmega = kMaxOmega / 2.0;
  public static final double kMaxAccelOmegaPath = 5.0; // 3.14
  public static final double kMaxAccelOmegaSpin = 8.0; // 3.14

  // Default safety path constants
  public static final Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d endPose2d = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
  public static final double kYawTuningTarget = 0;

  public static ArrayList<Translation2d> getDefaultInternalWaypoints() {
    ArrayList<Translation2d> waypoints = new ArrayList<>();
    waypoints.add(new Translation2d(0.5, 0));
    return waypoints;
  }

  public static TrajectoryConfig getDefaultTrajectoryConfig() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 1);
    trajectoryConfig.setReversed(true);
    trajectoryConfig.setStartVelocity(0.0);
    trajectoryConfig.setEndVelocity(0.0);
    return trajectoryConfig;
  }
}
