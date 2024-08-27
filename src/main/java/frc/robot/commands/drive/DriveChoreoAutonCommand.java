// package frc.robot.commands.drive;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.auto.AutoCommandInterface;
// import frc.robot.subsystems.drive.DriveSubsystem;
// import frc.robot.subsystems.drive.PathData;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;

// public class DriveChoreoAutonCommand extends Command implements AutoCommandInterface {
//   private final DriveSubsystem driveSubsystem;
//   private ChoreoTrajectory trajectory;
//   private final Timer timer = new Timer();
//   private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
//   private Rotation2d robotHeading;
//   private boolean lastPath;
//   private String trajectoryName;
//   private boolean resetOdometry;

//   public DriveChoreoAutonCommand(
//       DriveSubsystem driveSubsystem,
//       String trajectoryName,
//       boolean resetOdometry,
//       boolean lastPath) {

//     addRequirements(driveSubsystem);
//     this.driveSubsystem = driveSubsystem;
//     this.lastPath = lastPath;
//     this.resetOdometry = resetOdometry;
//     this.trajectoryName = trajectoryName;
//     this.trajectory = Choreo.getTrajectory(trajectoryName);
//     timer.start();
//   }

//   public void generateTrajectory() {
//     //I don't have to do anything here since it's a choreo path
//   }

//   @Override
//   public boolean hasGenerated() {
//     return true;
//   }

//   @Override
//   public void initialize() {
//     driveSubsystem.setEnableHolo(true);
//     // driveSubsystem.recordAutoTrajectory(trajectory); // TODO I don't know how important this
// is
//     Pose2d initialPose = trajectory.getInitialPose();
//     if (resetOdometry)
//       driveSubsystem.resetOdometry(
//           new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
//     driveSubsystem.resetHolonomicController();
//     driveSubsystem.grapherTrajectoryActive(true);
//     timer.reset();
//     logger.info("Begin Trajectory: {}", trajectoryName);
//     Trajectory.State desiredState = trajectory.sample(timer.get());
//     driveSubsystem.calculateController(desiredState, robotHeading);
//   }

//   @Override
//   public void execute() {
//     Trajectory.State desiredState = trajectory.sample(timer.get());
//     driveSubsystem.calculateController(desiredState, robotHeading);
//   }

//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(trajectory.getTotalTime());
//   }

//   @Override
//   public void end(boolean interrupted) {
//     driveSubsystem.setEnableHolo(false);
//     driveSubsystem.recordAutoTrajectory(null);

//     if (!lastPath) {
//       driveSubsystem.calculateController(
//           trajectory.sample(trajectory.getTotalTime()), robotHeading);
//     } else {
//       driveSubsystem.drive(0, 0, 0);
//     }

//     driveSubsystem.grapherTrajectoryActive(false);
//     logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
//   }
// }
