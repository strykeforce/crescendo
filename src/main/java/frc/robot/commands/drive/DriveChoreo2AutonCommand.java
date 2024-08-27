package frc.robot.commands.drive;

import com.choreo.lib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveChoreo2AutonCommand extends Command implements AutoCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveChoreo2AutonCommand.class);
  private Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private ChoreoTrajectory traj;
  private Command choreoSwerveCommand;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;

  public DriveChoreo2AutonCommand(
      DriveSubsystem driveSubsystem, String trajectoryName, boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.resetOdometry = resetOdometry;
    this.trajectoryName = trajectoryName;
    timer.start();

    traj = Choreo.getTrajectory(trajectoryName); //

    this.choreoSwerveCommand =
        Choreo.choreoSwerveCommand(
            traj,
            () -> driveSubsystem.getPoseMeters,
            driveSubsystem.getxController(),
            driveSubsystem.getyController(),
            driveSubsystem.getomegaControllerNonProfiled(),
            (ChassisSpeeds speeds) ->
                this.drive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond),
            () -> {
              driveSubsystem.shouldFlip();
            },
            driveSubsystem);
  }

  public void generateTrajectory() {}

  @Override
  public boolean hasGenerated() {
    return true;
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.recordAutoTrajectory(trajectory);
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry)
      driveSubsystem.resetOdometry(
          new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory: {}", trajectoryName);
    choreoSwerveCommand.initialize();
  }

  @Override
  public void execute() {
    choreoSwerveCommand.execute();
  }

  @Override
  public boolean isFinished() {
    choreoSwerveCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.recordAutoTrajectory(null);

    if (!lastPath) {
      choreoSwerveCommand.end(interrupted);
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    trajectoryGenerated = false;
  }
}
