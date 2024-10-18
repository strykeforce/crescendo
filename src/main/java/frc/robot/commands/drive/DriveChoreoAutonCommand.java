package frc.robot.commands.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveChoreoAutonCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private ChoreoTrajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private String trajectoryName;
  private boolean mirrorTrajectory = false;

  private boolean resetOdometry;
  private boolean lastPath;

  public DriveChoreoAutonCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.resetOdometry = resetOdometry;
    this.lastPath = lastPath;
    this.trajectoryName = trajectoryName;
    generateTrajectory();
    trajectory = Choreo.getTrajectory(trajectoryName);
    timer.start();
  }

  public void generateTrajectory() {
    mirrorTrajectory = driveSubsystem.shouldFlip();
  }

  @Override
  public boolean hasGenerated() {
    return true;
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    // driveSubsystem.recordAutoTrajectory(trajectory);
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry) {
      driveSubsystem.resetOdometry(initialPose);
    }
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory: {}", trajectoryName);
    ChoreoTrajectoryState desiredState = trajectory.sample(timer.get(), mirrorTrajectory);
    driveSubsystem.calculateController(desiredState);
  }

  @Override
  public void execute() {
    ChoreoTrajectoryState desiredState = trajectory.sample(timer.get(), mirrorTrajectory);
    driveSubsystem.calculateController(desiredState);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    // driveSubsystem.recordAutoTrajectory(null);

    if (!interrupted && !lastPath) {
      driveSubsystem.calculateController(
          trajectory.sample(trajectory.getTotalTime(), mirrorTrajectory));
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
  }
}
