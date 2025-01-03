package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;

  public DriveAutonCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    this.trajectoryName = trajectoryName;
    generateTrajectory();
    timer.start();
  }

  public void generateTrajectory() {
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;

    driveSubsystem.logTrajectory(trajectory);

    logger.info("trajectory generated");
    trajectoryGenerated = true;
  }

  @Override
  public boolean hasGenerated() {
    return trajectoryGenerated;
  }

  @Override
  public void initialize() {
    driveSubsystem.setAutoDebugMsg("Initialize " + trajectoryName);
    driveSubsystem.setEnableHolo(true);
    // driveSubsystem.recordAutoTrajectory(trajectory);
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
      driveSubsystem.calculateController(desiredState, robotHeading);
    } else logger.error("trajectory not generated");
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.recordAutoTrajectory(null);

    if (!lastPath) {
      driveSubsystem.calculateController(
          trajectory.sample(trajectory.getTotalTimeSeconds()), robotHeading);
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    driveSubsystem.setAutoDebugMsg("End " + trajectoryName);
    trajectoryGenerated = false;
  }
}
