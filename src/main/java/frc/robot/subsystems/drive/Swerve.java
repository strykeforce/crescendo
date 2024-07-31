package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.strykeforce.gyro.SF_PIGEON2;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.Measurable;
import org.strykeforce.telemetry.measurable.Measure;

public class Swerve implements SwerveIO, Checkable, Measurable {
  @HealthCheck private final SwerveDrive swerveDrive;

  // Grapher stuff
  private PoseEstimatorOdometryStrategy odometryStrategy;

  private SF_PIGEON2 pigeon;

  private AHRS ahrs;

  private TalonFXConfigurator configurator;

  private BooleanSupplier azimuth1FwdLimitSupplier = () -> false;

  private TalonSRX[] azimuths = new TalonSRX[4];
  private TalonFX[] drives = new TalonFX[4];

  private V6TalonSwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  private double fieldY = 0.0;
  private double fieldX = 0.0;

  public Swerve() {

    var moduleBuilder =
        new V6TalonSwerveModule.V6Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(RobotConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond)
            .latencyCompensation(true);

    swerveModules = new V6TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuths[i] = azimuthTalon;
      azimuthTalon.configFactoryDefault(RobotConstants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          DriveConstants.getAzimuthTalonConfig(), RobotConstants.kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      if (i == 1)
        azimuth1FwdLimitSupplier =
            () -> azimuthTalon.getSensorCollection().isFwdLimitSwitchClosed();

      var driveTalon = new TalonFX(i + 10);
      drives[i] = driveTalon;
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration()); // factory default
      configurator.apply(DriveConstants.getDriveTalonConfig());
      driveTalon.getSupplyVoltage().setUpdateFrequency(100);
      driveTalon.getSupplyCurrent().setUpdateFrequency(100);
      driveTalon.getClosedLoopReference().setUpdateFrequency(200);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }

    pigeon = new SF_PIGEON2(DriveConstants.kPigeonCanID, "*");
    ahrs = new AHRS(SPI.Port.kMXP, 2_000_000, (byte) 200);
    pigeon.applyConfig(DriveConstants.getPigeon2Configuration());
    swerveDrive = new SwerveDrive(false, 0.02, pigeon, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    kinematics = swerveDrive.getKinematics();

    odometryStrategy =
        new PoseEstimatorOdometryStrategy(
            swerveDrive.getHeading(),
            new Pose2d(),
            swerveDrive.getKinematics(),
            VisionConstants.kStateStdDevs,
            VisionConstants.kLocalMeasurementStdDevs,
            VisionConstants.kVisionMeasurementStdDevs,
            getSwerveModulePositions());

    swerveDrive.setOdometry(odometryStrategy);
  }

  // Getters/Setter
  @Override
  public String getName() {
    return "Swerve";
  }

  public SwerveModule[] getSwerveModules() {
    return swerveDrive.getSwerveModules();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModule[] swerveModules = getSwerveModules();
    SwerveModulePosition[] temp = {null, null, null, null};
    for (int i = 0; i < 4; ++i) {
      temp[i] = swerveModules[i].getPosition();
    }
    return temp;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    V6TalonSwerveModule[] swerveModules = (V6TalonSwerveModule[]) swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    return swerveModuleStates;
  }

  public ChassisSpeeds getRobotRelSpeed() {
    SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    return kinematics.toChassisSpeeds(swerveModuleStates);
  }

  public ChassisSpeeds getFieldRelSpeed() {
    // SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
    // SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    ChassisSpeeds roboRelSpeed = kinematics.toChassisSpeeds(swerveModuleStates);

    Rotation2d heading = swerveDrive.getHeading().unaryMinus();
    fieldX =
        roboRelSpeed.vxMetersPerSecond * heading.getCos()
            + roboRelSpeed.vyMetersPerSecond * heading.getSin();
    fieldY =
        -roboRelSpeed.vxMetersPerSecond * heading.getSin()
            + roboRelSpeed.vyMetersPerSecond * heading.getCos();

    return new ChassisSpeeds(fieldX, fieldY, roboRelSpeed.omegaRadiansPerSecond);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.getKinematics();
  }

  public void setOdometry(Rotation2d Odom) {
    swerveDrive.setOdometry(null);
  }

  public void setGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  public BooleanSupplier getAzimuth1FwdLimitSwitch() {
    return azimuth1FwdLimitSupplier;
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
    ahrs.reset();
  }

  public void updateSwerve() {
    swerveDrive.periodic();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    odometryStrategy.addVisionMeasurement(pose, timestamp);
  }

  public void addVisionMeasurement(Pose2d pose2d, double timestamp, Matrix<N3, N1> stdDevs) {
    odometryStrategy.addVisionMeasurement(pose2d, timestamp, stdDevs);
  }

  public void drive(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.drive(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void setAzimuthVel(double vel) {
    for (int i = 0; i < 4; i++) {
      azimuths[i].set(TalonSRXControlMode.PercentOutput, vel);
    }
  }

  public void configDriveCurrents(CurrentLimitsConfigs config) {
    for (int i = 0; i < 4; i++) {
      drives[i].getConfigurator().apply(config);
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.odometryX = swerveDrive.getPoseMeters().getX();
    inputs.odometryY = swerveDrive.getPoseMeters().getY();
    inputs.odometryRotation2D = swerveDrive.getPoseMeters().getRotation().getDegrees();
    inputs.gyroRotation2d = swerveDrive.getHeading();
    inputs.gyroPitch = pigeon.getPitch();
    inputs.gyroRoll = pigeon.getRoll();
    inputs.gyroRate = swerveDrive.getGyroRate();
    inputs.navXRotation2d = ahrs.getRotation2d();
    inputs.navXPitch = ahrs.getPitch();
    inputs.navXRoll = ahrs.getRoll();
    inputs.navXRate = ahrs.getRate();
    inputs.isConnected = pigeon.getPigeon2().getUpTime().hasUpdated();
    inputs.poseMeters = swerveDrive.getPoseMeters();
    inputs.updateCount = pigeon.getPigeon2().getTemperature().getValueAsDouble();
    inputs.fieldY = fieldY;
    inputs.fieldX = fieldX;
    for (int i = 0; i < 4; ++i) {
      inputs.azimuthVels[i] = azimuths[i].getSelectedSensorVelocity();
      inputs.azimuthCurrent[i] = azimuths[i].getSupplyCurrent();
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
    pigeon.registerWith(telemetryService);
    this.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("navXPitch", () -> ahrs.getRotation2d().getDegrees()),
        new Measure("navXPitch", () -> ahrs.getPitch()),
        new Measure("navXPitch", () -> ahrs.getRoll()),
        new Measure("navXPitch", () -> ahrs.getRate()));
  }

  @Override
  public String getDescription() {
    return "Swerve Drive";
  }

  @Override
  public int getDeviceId() {
    return 100;
  }
}
