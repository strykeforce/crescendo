package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
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
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import java.util.function.BooleanSupplier;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;
import org.strykeforce.telemetry.TelemetryService;

public class Swerve implements SwerveIO {

  private final SwerveDrive swerveDrive;

  // Grapher stuff
  private PoseEstimatorOdometryStrategy odometryStrategy;

  private SF_AHRS ahrs;

  private TalonFXConfigurator configurator;

  private BooleanSupplier azimuth1FwdLimitSupplier = () -> false;

  public Swerve() {

    var moduleBuilder =
        new V6TalonSwerveModule.V6Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(RobotConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    V6TalonSwerveModule[] swerveModules = new V6TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
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
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration()); // factory default
      configurator.apply(DriveConstants.getDriveTalonConfig());
      driveTalon.getClosedLoopReference().setUpdateFrequency(100);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }

    ahrs = new SF_AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
    swerveDrive = new SwerveDrive(ahrs, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

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

  public ChassisSpeeds getFieldRelSpeed() {
    SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    ChassisSpeeds roboRelSpeed = kinematics.toChassisSpeeds(swerveModuleStates);
    return new ChassisSpeeds(
        roboRelSpeed.vxMetersPerSecond * swerveDrive.getHeading().unaryMinus().getCos()
            + roboRelSpeed.vyMetersPerSecond * swerveDrive.getHeading().unaryMinus().getSin(),
        -roboRelSpeed.vxMetersPerSecond * swerveDrive.getHeading().unaryMinus().getSin()
            + roboRelSpeed.vyMetersPerSecond * swerveDrive.getHeading().unaryMinus().getCos(),
        roboRelSpeed.omegaRadiansPerSecond);
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

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.odometryX = swerveDrive.getPoseMeters().getX();
    inputs.odometryY = swerveDrive.getPoseMeters().getY();
    inputs.odometryRotation2D = swerveDrive.getPoseMeters().getRotation().getDegrees();
    inputs.gyroRotation2d = swerveDrive.getHeading();
    inputs.gyroPitch = ahrs.getPitch();
    inputs.gyroRoll = ahrs.getRoll();
    inputs.gyroRate = swerveDrive.getGyroRate();
    inputs.isConnected = ahrs.isConnected();
    inputs.poseMeters = swerveDrive.getPoseMeters();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
  }
}
