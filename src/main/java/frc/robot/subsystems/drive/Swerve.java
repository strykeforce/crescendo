package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;

public class Swerve implements SwerveIO {

  private final SwerveDrive swerveDrive;

  // Grapher stuff
  private PoseEstimatorOdometryStrategy odometryStrategy;

  private AHRS ahrs;

  public Swerve() {
    var moduleBuilder =
        new TalonSwerveModule.Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          DriveConstants.getAzimuthTalonConfig(), Constants.kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonFX(i + 10);
      driveTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      driveTalon.configAllSettings(
          DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      // var driveTalonFollower = new TalonFX(i + 14);
      // driveTalonFollower.configFactoryDefault(Constants.kTalonConfigTimeout);
      // driveTalonFollower.configAllSettings(
      //     DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
      // driveTalonFollower.enableVoltageCompensation(true);
      // driveTalonFollower.setNeutralMode(NeutralMode.Brake);
      // driveTalonFollower.follow(driveTalon);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();

      // if (i == 0) followerZero = driveTalon;
      // else if (i == 1) followerOne = driveTalon;
      // else if (i == 2) followerTwo = driveTalon;
      // else if (i == 3) followerThree = driveTalon;
    }

    ahrs = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
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

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
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
    TalonSwerveModule[] swerveModules = (TalonSwerveModule[]) swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    return swerveModuleStates;
  }

  public Rotation2d getGyroRotation2d() {
    return swerveDrive.getHeading();
  }

  public double getGyroPitch() {
    return ahrs.getPitch();
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public double getGyroRoll() {
    return ahrs.getRoll();
  }

  public void periodic() {
    swerveDrive.periodic();
  }

  public boolean isConnected() {
    return ahrs.isConnected();
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

  public void setGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void setOdometry(Rotation2d Odom) {
    swerveDrive.setOdometry(null);
  }

  public Pose2d getPoseMeters() {
    return swerveDrive.getPoseMeters();
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.odometryX = swerveDrive.getPoseMeters().getX();
    inputs.odometryY = swerveDrive.getPoseMeters().getY();
    inputs.gyroRotation = getGyroRotation2d().getDegrees();
    inputs.odometryRotation2D = swerveDrive.getPoseMeters().getRotation().getDegrees();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
    // telemetryService.register(followerZero);
    // telemetryService.register(followerOne);
    // telemetryService.register(followerTwo);
    // telemetryService.register(followerThree);
  }
}