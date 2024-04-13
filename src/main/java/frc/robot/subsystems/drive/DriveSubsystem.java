package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.AutonConstants.Setpoints;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;
import net.jafama.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveIO io;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private final HolonomicDriveController holonomicController;
  private RobotStateSubsystem robotStateSubsystem;

  private final ProfiledPIDController omegaShootTrackController;
  private final ProfiledPIDController omegaSpinController;
  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;

  public DriveStates currDriveState = DriveStates.IDLE;

  private double accelX = 0;
  private double accelY = 0;
  private double prevVelX = 0;
  private double prevVelY = 0;
  private double prevTimeStamp = Timer.getFPGATimestamp();
  private int velXStableCounts = 0;
  private int velYStableCounts = 0;
  private Pose2d moveAndShootVirtualPose = new Pose2d();

  // Grapher stuff
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();
  private State holoContInput = new State();
  private Rotation2d holoContAngle = new Rotation2d();
  private double trajectoryActive = 0.0;
  private double[] lastVelocity = new double[3];
  private boolean isAligningShot = false;
  private boolean isFeeding = false;
  private boolean deadEYEAutoDrive = false;
  private boolean isMoveAndShoot = false;

  private boolean tuningYaw = false;
  private double trackingSetpoint = 0.0;

  private boolean updateVision = true;

  public DriveSubsystem(SwerveIO io) {
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/YVelSpeed", 0.0);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/UsingDeadEye", false);
    this.io = io;

    // Setup omega Controller
    omegaShootTrackController =
        new ProfiledPIDController(
            DriveConstants.kPOmegaSpin,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaSpin));
    omegaShootTrackController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    omegaSpinController =
        new ProfiledPIDController(
            DriveConstants.kPOmegaSpin,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaSpin));
    omegaSpinController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    // Setup Holonomic Controller
    omegaController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaPath));
    omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    xController =
        new PIDController(
            DriveConstants.kXPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // xController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    yController =
        new PIDController(
            DriveConstants.kYPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // yController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    holonomicController = new HolonomicDriveController(xController, yController, omegaController);
    // Disabling the holonomic controller makes the robot directly follow the
    // trajectory output (no
    // closing the loop on x,y,theta errors)
    holonomicController.setEnabled(true);
  }

  // Open-Loop Swerve Movements
  public void drive(double vXmps, double vYmps, double vOmegaRadps) {
    if (!isAligningShot && !tuningYaw) {
      io.drive(vXmps, vYmps, vOmegaRadps, true);
    } else {
      double vOmegaRadpsNew;
      if (isFeeding) vOmegaRadpsNew = getvOmegaToFeedTarget();
      else if (isMoveAndShoot) vOmegaRadpsNew = getvOmegaToGoal(moveAndShootVirtualPose);
      else if (tuningYaw) {
        vOmegaRadpsNew = getvOmegaToGoal();
      } else vOmegaRadpsNew = getvOmegaToGoal();
      io.move(
          vXmps * (isMoveAndShoot ? DriveConstants.kMoveShootVelDetune : 1.0),
          vYmps * (isMoveAndShoot ? DriveConstants.kMoveShootVelDetune : 1.0),
          vOmegaRadpsNew,
          true);
    }
  }

  public double getvOmegaToGoal() {
    return omegaShootTrackController.calculate(
        getPoseMeters().getRotation().getRadians(),
        getPoseMeters().getRotation().getRadians() + getShooterAngleToSpeaker().getRadians());
  }

  public double getvOmegaToFeedTarget() {
    return omegaSpinController.calculate(
        getPoseMeters().getRotation().getRadians(),
        getPoseMeters().getRotation().getRadians() + getShooterAngleToFeedTarget().getRadians());
  }

  public double getvOmegaToTarget(Rotation2d target) {
    holoContAngle = target;
    return omegaSpinController.calculate(
        getPoseMeters().getRotation().getRadians(), target.getRadians());
  }

  public double getvOmegaToGoal(Pose2d pos) {
    trackingSetpoint = pos.getRotation().getRadians() + getShooterAngleToSpeaker(pos).getRadians();
    return omegaShootTrackController.calculate(pos.getRotation().getRadians(), trackingSetpoint);
  }

  // Closed-Loop (Velocity Controlled) Swerve Movement
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    io.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void recordYVel(double val) {
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/YVelSpeed", val);
  }

  public void setDeadEyeDrive(boolean val) {
    deadEYEAutoDrive = val;
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/UsingDeadEye", deadEYEAutoDrive);
  }

  // Holonomic Controller
  public void calculateController(State desiredState, Rotation2d desiredAngle) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput = holonomicController.calculate(inputs.poseMeters, desiredState, desiredAngle);
    // logger.info("input: {}, output: {}, angle: {}", holoContInput,
    // holoContOutput, desiredAngle);
    io.move(
        holoContOutput.vxMetersPerSecond,
        holoContOutput.vyMetersPerSecond,
        holoContOutput.omegaRadiansPerSecond,
        false);
  }

  public void driveAutonXController(State desiredState, Rotation2d desiredAngle, double driveY) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput = holonomicController.calculate(inputs.poseMeters, desiredState, desiredAngle);
    // logger.info("input: {}, output: {}, angle: {}", holoContInput,
    // holoContOutput, desiredAngle);
    io.move(holoContOutput.vxMetersPerSecond, driveY, holoContOutput.omegaRadiansPerSecond, false);
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
    logger.info("reset odometry with: {}", pose);
  }

  public boolean usingVisionUpdates() {
    return updateVision;
  }

  public void enableVisionUpdates(boolean val) {
    updateVision = val;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    io.addVisionMeasurement(pose, timestamp);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevvs) {
    io.addVisionMeasurement(pose, timestamp, stdDevvs);
  }

  public void setOmegaKP(double kP, double accel) {
    holonomicController.getThetaController().setP(kP);
    holonomicController
        .getThetaController()
        .setConstraints(new Constraints(DriveConstants.kMaxOmega, accel));
  }

  public void resetHolonomicController(double yaw) {
    xController.reset();
    yController.reset();
    omegaController.reset(yaw);
    omegaSpinController.reset(yaw);
    omegaShootTrackController.reset(yaw);
    holonomicController.getThetaController().reset(yaw);
    logger.info("Holonomic Controller Reset: {}", yaw);
  }

  public void resetHolonomicController() {
    xController.reset();
    yController.reset();
    omegaController.reset(inputs.gyroRotation2d.getRadians());
    omegaSpinController.reset(inputs.gyroRotation2d.getRadians());
    omegaShootTrackController.reset(inputs.gyroRotation2d.getRadians());
    holonomicController.getThetaController().reset(inputs.gyroRotation2d.getRadians());
    logger.info("Holonomic Controller Reset: {}", inputs.gyroRotation2d.getRadians());
  }

  public void setHolonomicControllerTranslationkP(double kP) {
    xController.setP(kP);
    yController.setP(kP);
  }

  public void resetOmegaController() {
    omegaController.reset(inputs.gyroRotation2d.getRadians());
    omegaSpinController.reset(inputs.gyroRotation2d.getRadians());
    omegaShootTrackController.reset(inputs.gyroRotation2d.getRadians());
  }

  // Getters/Setters
  public Pose2d getPoseMeters() {
    return inputs.poseMeters;
  }

  public Rotation2d getGyroRotation2d() {
    return inputs.gyroRotation2d;
  }

  public ChassisSpeeds getFieldRelSpeed() {
    return io.getFieldRelSpeed();
  }

  public ChassisSpeeds getRobotRelSpeed() {
    return io.getRobotRelSpeed();
  }

  public Translation2d getShooterPos() {
    Pose2d pose = getPoseMeters();
    Translation2d shooterOffset =
        new Translation2d(-RobotConstants.kShooterOffset, pose.getRotation());

    return pose.getTranslation().plus(shooterOffset);
  }

  public Translation2d getShooterPos(Pose2d pos) {
    Pose2d pose = pos;
    Translation2d shooterOffset =
        new Translation2d(-RobotConstants.kShooterOffset, pose.getRotation());

    return pose.getTranslation().plus(shooterOffset);
  }

  public double getDistanceToFeedTarget() {
    return getShooterPos()
        .getDistance(
            robotStateSubsystem.getAllianceColor() == Alliance.Blue
                ? RobotConstants.kBlueFeedTargetPos
                : RobotConstants.kRedFeedTargetPos);
  }

  public double getDistanceToSpeaker() {
    return getShooterPos()
        .getDistance(
            robotStateSubsystem.getAllianceColor() == Alliance.Blue
                ? RobotConstants.kBlueSpeakerPos
                : RobotConstants.kRedSpeakerPos);
  }

  public double getDistanceToSpeaker(Pose2d pos) {
    return getShooterPos(pos)
        .getDistance(
            robotStateSubsystem.getAllianceColor() == Alliance.Blue
                ? RobotConstants.kBlueSpeakerPos
                : RobotConstants.kRedSpeakerPos);
  }

  public void setMoveAndShootVirtualPose(Pose2d virtualPose) {
    org.littletonrobotics.junction.Logger.recordOutput("ShootingData/FuturePos", virtualPose);
    this.moveAndShootVirtualPose = virtualPose;
  }

  public Rotation2d getShooterAngleToFeedTarget() {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue)
      return RobotConstants.kBlueFeedTargetPos
          .minus(getPoseMeters().getTranslation())
          .getAngle()
          .minus(getPoseMeters().getRotation().rotateBy(RobotConstants.kShooterHeading))
          .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
    return RobotConstants.kRedFeedTargetPos
        .minus(getPoseMeters().getTranslation())
        .getAngle()
        .minus(getPoseMeters().getRotation().rotateBy(RobotConstants.kShooterHeading))
        .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
  }

  public Rotation2d getShooterAngleToSpeaker() {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue)
      return RobotConstants.kBlueSpeakerPos
          .minus(getPoseMeters().getTranslation())
          .getAngle()
          .minus(getPoseMeters().getRotation().rotateBy(RobotConstants.kShooterHeading))
          .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
    return RobotConstants.kRedSpeakerPos
        .minus(getPoseMeters().getTranslation())
        .getAngle()
        .minus(getPoseMeters().getRotation().rotateBy(RobotConstants.kShooterHeading))
        .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
  }

  public Rotation2d getShooterAngleToSpeaker(Pose2d pos) {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue)
      return RobotConstants.kBlueSpeakerPos
          .minus(pos.getTranslation())
          .getAngle()
          .minus(pos.getRotation().rotateBy(RobotConstants.kShooterHeading))
          .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
    return RobotConstants.kRedSpeakerPos
        .minus(pos.getTranslation())
        .getAngle()
        .minus(pos.getRotation().rotateBy(RobotConstants.kShooterHeading))
        .rotateBy(new Rotation2d(RobotConstants.kDegreeShootOffset));
  }

  public DriveStates getDriveState() {
    return currDriveState;
  }

  public BooleanSupplier getAzimuth1FwdLimitSupplier() {
    return io.getAzimuth1FwdLimitSwitch();
  }

  public boolean isPointingAtFeedTarget() {
    return Math.abs(getShooterAngleToFeedTarget().getDegrees())
        <= DriveConstants.kDegreesCloseEnough;
  }

  public boolean isPointingAtGoal() {
    return Math.abs(getShooterAngleToSpeaker().getDegrees()) <= DriveConstants.kDegreesCloseEnough;
  }

  public boolean isDriveStillFeed() {
    double vX = getFieldRelSpeed().vxMetersPerSecond;
    double vY = getFieldRelSpeed().vyMetersPerSecond;

    // Take fieldRel Speed and get the magnitude of the vector
    double wheelSpeed = FastMath.hypot(vX, vY);

    double gyroRate = inputs.gyroRate;

    boolean velStill = Math.abs(wheelSpeed) <= DriveConstants.kSpeedStillFeedThreshold;
    boolean gyroStill = Math.abs(gyroRate) <= DriveConstants.kGyroRateStillThreshold;

    return velStill && gyroStill;
  }

  public boolean isPointingAtGoal(Pose2d pos) {
    return Math.abs(getShooterAngleToSpeaker(pos).getDegrees())
        <= (isFeeding
            ? DriveConstants.kDegreesCloseEnoughFeeding
            : DriveConstants.kDegreesCloseEnough);
  }

  public boolean isDriveStill() {
    // logger.info(
    //     "Timestamp Before FieldRel: {}",
    //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
    ChassisSpeeds cs = getFieldRelSpeed();
    double vX = cs.vxMetersPerSecond;
    double vY = cs.vyMetersPerSecond;
    // logger.info(
    //     "Timestamp After FieldRel: {}",
    //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);

    // Take fieldRel Speed and get the magnitude of the vector
    double wheelSpeed = FastMath.hypot(vX, vY);

    double gyroRate = inputs.gyroRate;

    boolean velStill = Math.abs(wheelSpeed) <= DriveConstants.kSpeedStillThreshold;
    boolean gyroStill = Math.abs(gyroRate) <= DriveConstants.kGyroRateStillThreshold;

    return velStill && gyroStill;
  }

  public boolean isMoveShootAllowed(boolean isAuto) {
    double vX = getFieldRelSpeed().vxMetersPerSecond;
    double vY = getFieldRelSpeed().vyMetersPerSecond;

    // Take fieldRel Speed and get the magnitude of the vector
    double wheelSpeed = FastMath.hypot(vX, vY);

    double gyroRate = inputs.gyroRate;

    boolean velStill =
        Math.abs(wheelSpeed) <= DriveConstants.kMaxMoveShootVelocity
            && Math.abs(velXStableCounts) <= DriveConstants.kVelocityStableCounts
            && Math.abs(velYStableCounts) <= DriveConstants.kVelocityStableCounts;
    boolean gyroStill = Math.abs(gyroRate) <= DriveConstants.kMaxMoveGyroRateThreshold;

    return velStill
        && gyroStill
        && getDistanceToSpeaker() <= DriveConstants.kMaxSpeakerDist
        && (isAuto || vX <= DriveConstants.kMoveShootTeleMaxVelX);
  }

  public boolean isNavxWorking() {
    return inputs.isConnected;
  }

  public double getNavxUpdateNum() {
    return inputs.updateCount;
  }

  public void setGyroOffset(Rotation2d rotation) {
    io.setGyroOffset(apply(rotation));
  }

  public void setEnableHolo(boolean enabled) {
    holonomicController.setEnabled(enabled);
    logger.info("Holonomic Controller Enabled: {}", enabled);
  }

  public void setIsAligningShot(boolean isAligningShot) {
    this.isAligningShot = isAligningShot;
    resetOmegaController();
  }

  public void setIsFeeding(boolean isFeeding) {
    this.isFeeding = isFeeding;
    resetOmegaController();
  }

  public void setIsMoveAndShoot(boolean isMoveAndShoot) {
    this.isMoveAndShoot = isMoveAndShoot;
    resetOmegaController();
  }

  public void setIsTuningYaw(boolean isTuningYaw) {
    this.tuningYaw = isTuningYaw;
    if (isTuningYaw) resetOmegaController();
  }

  public boolean getIsTuningYaw() {
    return this.tuningYaw;
  }

  public void setDriveState(DriveStates driveStates) {
    logger.info("{} -> {}", currDriveState, driveStates);
    currDriveState = driveStates;
  }

  public void setRobotStateSubsystem(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  public void teleResetGyro() {
    logger.info("Driver Joystick: Reset Gyro");
    double gyroResetDegs = robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : 180.0;
    io.setGyroOffset(Rotation2d.fromDegrees(gyroResetDegs));
    io.resetGyro();
    io.resetOdometry(
        new Pose2d(inputs.poseMeters.getTranslation(), Rotation2d.fromDegrees(gyroResetDegs)));
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(Boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  // Field flipping stuff
  public boolean shouldFlip() {
    return robotStateSubsystem.getAllianceColor() == Alliance.Red;
  }

  public Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(DriveConstants.kFieldMaxX - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  public Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          DriveConstants.kFieldMaxX - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  public Rotation2d apply(Rotation2d rotation) {
    logger.info(
        "initial target yaw: {}, cos: {}, sin: {}", rotation, rotation.getCos(), rotation.getSin());
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else return rotation;
  }

  public double apply(double x) {
    logger.info("initial x: {}", x);
    if (shouldFlip()) {
      return DriveConstants.kFieldMaxX - x;
    } else return x;
  }

  // Trajectory TOML Parsing
  public PathData generateTrajectory(String trajectoryName) {
    try {
      if (shouldFlip()) {
        logger.info("Flipping path");
      }

      TomlParseResult parseResult =
          Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));

      logger.info("Generating Trajectory: {}", trajectoryName);

      Pose2d startPose = parseEndPoint(parseResult, "start_pose");
      Pose2d endPose = parseEndPoint(parseResult, "end_pose");

      startPose = apply(startPose);
      endPose = apply(endPose);

      TomlArray internalPointsToml = parseResult.getArray("internal_points");
      ArrayList<Translation2d> path = new ArrayList<>();

      logger.info("Toml Internal Points Array Size: {}", internalPointsToml.size());

      // Create a table for each internal point and put them into a translation2d
      // waypoint
      for (int i = 0; i < internalPointsToml.size(); i++) {
        TomlTable waypointToml = internalPointsToml.getTable(i);
        Translation2d waypoint =
            new Translation2d(waypointToml.getDouble("x"), waypointToml.getDouble("y"));
        waypoint = apply(waypoint);
        path.add(waypoint);
      }

      // Trajectory Config parsed from toml - any additional constraints would be
      // added here
      TrajectoryConfig trajectoryConfig =
          new TrajectoryConfig(
              parseResult.getDouble("max_velocity"), parseResult.getDouble("max_acceleration"));
      logger.info("max velocity/acceleration worked");

      trajectoryConfig.setReversed(parseResult.getBoolean("is_reversed"));
      trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
      logger.info("start velocity worked");

      trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));
      logger.info("end velocity worked");

      // Yaw degrees is seperate from the trajectoryConfig
      double yawDegrees = parseResult.getDouble("target_yaw");
      logger.info("target yaw worked");
      Rotation2d target_yaw = Rotation2d.fromDegrees(yawDegrees);
      target_yaw = apply(target_yaw);
      logger.info("Yaw is {}", target_yaw);

      // Create a the generated trajectory and return it along with the target yaw
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

      logger.info("Total Time: {}", trajectoryGenerated.getTotalTimeSeconds());
      return new PathData(target_yaw, trajectoryGenerated);
    } catch (Exception error) {
      logger.error(error.toString());
      logger.error("Path {} not found - Running Default Path", trajectoryName);

      // In the case of an error, set the trajectory to default
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(
              DriveConstants.startPose2d,
              DriveConstants.getDefaultInternalWaypoints(),
              DriveConstants.endPose2d,
              DriveConstants.getDefaultTrajectoryConfig());

      return new PathData(inputs.gyroRotation2d, trajectoryGenerated);
    }
  }

  private Pose2d parseEndPoint(TomlParseResult parseResult, String poseName) {
    TomlTable table = parseResult.getTable(poseName);

    if (table.contains("dataPoint")) {
      Pose2d pose;

      switch (table.getString("dataPoint")) {
          // Starting Positions
        case "MI1":
          pose = Setpoints.MI1;
          break;
        case "NAI1":
          pose = Setpoints.NAI1;
          break;
        case "AI1":
          pose = Setpoints.AI1;
          break;
        case "AI2":
          pose = Setpoints.AI2;
          break;
        case "AI3":
          pose = Setpoints.AI3;
          break;

          // Wing Notes
        case "W1":
          pose = Setpoints.W1;
          break;
        case "W2":
          pose = Setpoints.W2;
          break;
        case "W3":
          pose = Setpoints.W3;
          break;

          // Middle Notes
        case "M1":
          pose = Setpoints.M1;
          break;
        case "M2":
          pose = Setpoints.M2;
          break;
        case "M3":
          pose = Setpoints.M3;
          break;
        case "M4":
          pose = Setpoints.M4;
          break;
        case "M5":
          pose = Setpoints.M5;
          break;

          // Shooting Positions
        case "AS1":
          pose = Setpoints.AS1;
          break;
        case "AS2":
          pose = Setpoints.AS2;
          break;
        case "AS3":
          pose = Setpoints.AS3;
          break;
        case "MS1":
          pose = Setpoints.MS1;
          break;
        case "NAS1":
          pose = Setpoints.NAS1;
          break;
        case "NAS2":
          pose = Setpoints.NAS2;
          break;
        case "MP1":
          pose = Setpoints.MP1;
          break;
        case "MP2":
          pose = Setpoints.MP2;
          break;

        default:
          logger.warn("Bad data point {}", table.getString("dataPoint"));
          return new Pose2d();
      }

      double angle = pose.getRotation().getDegrees();
      double X = pose.getX();
      double Y = pose.getY();

      if (table.contains("angle")) {
        angle = table.getDouble("angle");
        logger.info("Changing angle to {}", angle);
      }

      if (table.contains("dX")) {
        X = X + table.getDouble("dX");
        logger.info("Changing X to {}", X);
      }

      if (table.contains("dY")) {
        Y = Y + table.getDouble("dY");
        logger.info("Changing Y to {}", Y);
      }
      pose = new Pose2d(X, Y, Rotation2d.fromDegrees(angle));

      return pose;
    } else {
      return new Pose2d(
          table.getDouble("x"),
          table.getDouble("y"),
          Rotation2d.fromDegrees(table.getDouble("angle")));
    }
  }

  // Control Methods
  public void lockZero() {
    SwerveModule[] swerveModules = io.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(0.0));
    }
  }

  public void xLock() {
    SwerveModule[] swerveModules = io.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      if (i == 0 || i == 3) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
      }

      if (i == 1 || i == 2) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Swerve", inputs);
    // Update swerve module states every robot loop
    io.updateSwerve();

    // Log Outputs FIXME
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Odometry", inputs.poseMeters);
    org.littletonrobotics.junction.Logger.recordOutput(
        "ShootingData/AngleToGoal", getShooterAngleToSpeaker().getDegrees());
    org.littletonrobotics.junction.Logger.recordOutput(
        "ShootingData/DistanceToGoal", getDistanceToSpeaker());

    // Compute acceleration
    accelX =
        (getFieldRelSpeed().vxMetersPerSecond - prevVelX)
            / (Timer.getFPGATimestamp() - prevTimeStamp);
    accelY =
        (getFieldRelSpeed().vyMetersPerSecond - prevVelY)
            / (Timer.getFPGATimestamp() - prevTimeStamp);

    if (Math.abs(accelX) < DriveConstants.kMaxStableAccel) {
      velXStableCounts++;
    } else {
      velXStableCounts = 0;
    }

    if (Math.abs(accelY) < DriveConstants.kMaxStableAccel) {
      velYStableCounts++;
    } else {
      velYStableCounts = 0;
    }

    prevVelX = getFieldRelSpeed().vxMetersPerSecond;
    prevVelY = getFieldRelSpeed().vyMetersPerSecond;
    prevTimeStamp = Timer.getFPGATimestamp();

    switch (currDriveState) {
      case IDLE:
        break;
      default:
        break;
    }
  }

  public enum DriveStates {
    IDLE,
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> currDriveState.ordinal()),
        new Measure("Gyro roll", () -> inputs.gyroRoll),
        new Measure("Gyro pitch", () -> inputs.gyroPitch),
        new Measure("Gyro Rotation2D(deg)", () -> inputs.gyroRotation2d.getDegrees()),
        new Measure("Odometry X", () -> inputs.poseMeters.getX()),
        new Measure("Odometry Y", () -> inputs.poseMeters.getY()),
        new Measure("Odometry Rotation2D(deg)", () -> inputs.poseMeters.getRotation().getDegrees()),
        new Measure("Trajectory Vel", () -> holoContInput.velocityMetersPerSecond),
        new Measure("Trajectory Accel", () -> holoContInput.accelerationMetersPerSecondSq),
        new Measure("Trajectory X", () -> holoContInput.poseMeters.getX()),
        new Measure("Trajectory Y", () -> holoContInput.poseMeters.getY()),
        new Measure(
            "Trajectory Rotation2D(deg)",
            () -> holoContInput.poseMeters.getRotation().getDegrees()),
        new Measure("Desired Gyro Heading(deg)", () -> holoContAngle.getDegrees()),
        new Measure("Holonomic Cont Vx", () -> holoContOutput.vxMetersPerSecond),
        new Measure("Holonomic Cont Vy", () -> holoContOutput.vyMetersPerSecond),
        new Measure("Holonomic Cont Vomega", () -> holoContOutput.omegaRadiansPerSecond),
        new Measure(
            "Holo Controller Omega Err",
            () -> holonomicController.getThetaController().getPositionError()),
        new Measure(
            "Holo Controller Omega Goal",
            () -> holonomicController.getThetaController().getGoal().position),
        new Measure(
            "Holo Controller Omega Setpoint",
            () -> holonomicController.getThetaController().getSetpoint().position),
        new Measure("Trajectory Active", () -> trajectoryActive),
        new Measure("Wheel 0 Angle", () -> io.getSwerveModuleStates()[0].angle.getDegrees()),
        new Measure("Wheel 0 Speed", () -> io.getSwerveModuleStates()[0].speedMetersPerSecond),
        new Measure("Wheel 1 Angle", () -> io.getSwerveModuleStates()[1].angle.getDegrees()),
        new Measure("Wheel 1 Speed", () -> io.getSwerveModuleStates()[1].speedMetersPerSecond),
        new Measure("Wheel 2 Angle", () -> io.getSwerveModuleStates()[2].angle.getDegrees()),
        new Measure("Wheel 2 Speed", () -> io.getSwerveModuleStates()[2].speedMetersPerSecond),
        new Measure("Wheel 3 Angle", () -> io.getSwerveModuleStates()[3].angle.getDegrees()),
        new Measure("Wheel 3 Speed", () -> io.getSwerveModuleStates()[3].speedMetersPerSecond),
        new Measure("FWD Vel", () -> lastVelocity[0]),
        new Measure("STR Vel", () -> lastVelocity[1]),
        new Measure("YAW Vel", () -> lastVelocity[2]),
        new Measure("Angle to goal", () -> getShooterAngleToSpeaker().getDegrees()),
        new Measure("Distance to goal", () -> getDistanceToSpeaker()),
        new Measure("Is Yaw Tuning", () -> getIsTuningYaw() ? 1 : 0),
        new Measure(
            "Angle For Goal",
            () ->
                (getPoseMeters().getRotation().getDegrees()
                    + getShooterAngleToSpeaker().getDegrees())),
        new Measure("MoveShoot Angle", () -> trackingSetpoint));
  }
}
