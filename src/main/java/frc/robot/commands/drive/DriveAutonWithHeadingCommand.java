package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonWithHeadingCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;
  private double xThreshold;
  private boolean towardsPosX;
  private Rotation2d startHeading;

  public DriveAutonWithHeadingCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry,
      Rotation2d startHeading,
      double xThreshold,
      boolean towardsPosX) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    this.trajectoryName = trajectoryName;
    this.xThreshold = xThreshold;
    this.towardsPosX = towardsPosX;
    this.startHeading = startHeading;

    generateTrajectory();
    timer.start();
  }

  public void generateTrajectory() {
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;
    startHeading = driveSubsystem.apply(startHeading);

    logger.info("trajectory generated");
    trajectoryGenerated = true;
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

      double currX = driveSubsystem.getPoseMeters().getX();

      if (driveSubsystem.shouldFlip()) {
        if (currX > xThreshold && towardsPosX || currX < xThreshold && !towardsPosX) {
          driveSubsystem.calculateController(desiredState, startHeading);
        } else {
          driveSubsystem.calculateController(desiredState, robotHeading);
        }

      } else {
        if (currX < xThreshold && towardsPosX || currX > xThreshold && !towardsPosX) {
          driveSubsystem.calculateController(desiredState, startHeading);
        } else {
          driveSubsystem.calculateController(desiredState, robotHeading);
        }
      }
    }
  }

  @Override
  public void execute() {
    if (trajectoryGenerated) {
      Trajectory.State desiredState = trajectory.sample(timer.get());
      double currX = driveSubsystem.getPoseMeters().getX();

      if (driveSubsystem.shouldFlip()) {
        if (currX > xThreshold && towardsPosX || currX < xThreshold && !towardsPosX) {
          driveSubsystem.calculateController(desiredState, startHeading);
        } else {
          driveSubsystem.calculateController(desiredState, robotHeading);
        }

      } else {
        if (currX < xThreshold && towardsPosX || currX > xThreshold && !towardsPosX) {
          driveSubsystem.calculateController(desiredState, startHeading);
        } else {
          driveSubsystem.calculateController(desiredState, robotHeading);
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
