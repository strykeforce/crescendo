package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveCenterLineCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(MiddleNoteDriveAutonCommand.class);
  private Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;
  private RobotStateSubsystem robotStateSubsystem;
  private ProfiledPIDController deadeyeYDrive;
  private DeadEyeSubsystem deadeye;
  private LedSubsystem ledSubsystem;

  public DriveCenterLineCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      DeadEyeSubsystem deadeye,
      LedSubsystem ledSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.deadeye = deadeye;
    this.ledSubsystem = ledSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    this.trajectoryName = trajectoryName;
    generateTrajectory();
    timer.start();
    deadeyeYDrive =
        new ProfiledPIDController(
            AutonConstants.kPDeadEyeYDrive,
            AutonConstants.kIDeadEyeYDrive,
            AutonConstants.kDDeadEyeYDrive,
            new Constraints(
                AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
  }

  public void generateTrajectory() {
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;
    logger.info("trajectory generated");
    trajectoryGenerated = true;
  }

  @Override
  public boolean hasGenerated() {
    return trajectoryGenerated;
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry)
      driveSubsystem.resetOdometry(
          new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory: {}", trajectoryName);
    if (trajectoryGenerated) {
      Trajectory.State desiredState = trajectory.sample(timer.get());
      driveSubsystem.calculateController(desiredState, robotHeading);
    }
  }

  @Override
  public void execute() {
    if (trajectoryGenerated) {
      Trajectory.State desiredState = trajectory.sample(timer.get());

      double yVel = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      driveSubsystem.recordYVel(yVel);
      driveSubsystem.driveAutonXController(desiredState, robotHeading, yVel);

      double currY = driveSubsystem.getPoseMeters().getY();

      if (robotStateSubsystem.getState() == RobotStates.AUTO_DISRUPT) {
        Alliance alliance = robotStateSubsystem.getAllianceColor();
        if (alliance == Alliance.Blue && currY > AutonConstants.kDisruptIntakingYBlue
            || alliance == Alliance.Red && currY > AutonConstants.kDisruptIntakingYRed) {
          robotStateSubsystem.toIntake();
        }
      }

    } else logger.error("trajectory not generated");
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.setDeadEyeDrive(false);

    if (!lastPath) {
      driveSubsystem.calculateController(
          trajectory.sample(trajectory.getTotalTimeSeconds()), robotHeading);
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    trajectoryGenerated = false;
  }
}
