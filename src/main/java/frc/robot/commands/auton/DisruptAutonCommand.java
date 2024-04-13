package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.intake.SetHasPieceCommand;
import frc.robot.commands.magazine.SetFullCommand;
import frc.robot.commands.robotState.PrepShooterCommand;
import frc.robot.commands.robotState.ToDisruptCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class DisruptAutonCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private boolean hasGenerated = false;

  private DriveAutonCommand firstPath;
  private MiddleNoteDriveAutonCommand secondPath;
  private DriveCenterLineCommand thirdPath;
  private DriveAutonCommand shootPath;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;

  private ProfiledPIDController deadeyeXDrive;

  public DisruptAutonCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      DeadEyeSubsystem deadeye,
      LedSubsystem ledSubsystem,
      String firstPathName,
      String secondPathName,
      String thirdPathName,
      String shootPathName,
      Pose2d shootPose,
      double startIntakingY) {
    addRequirements(
        driveSubsystem, superStructure, magazineSubsystem, intakeSubsystem, elbowSubsystem);
    firstPath = new DriveAutonCommand(driveSubsystem, firstPathName, false, true);
    secondPath =
        new MiddleNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            secondPathName,
            false,
            false);

    thirdPath =
        new DriveCenterLineCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            thirdPathName,
            startIntakingY,
            false,
            false);

    shootPath = new DriveAutonCommand(driveSubsystem, shootPathName, true, false);

    this.robotStateSubsystem = robotStateSubsystem;

    deadeyeXDrive =
        new ProfiledPIDController(
            AutonConstants.kPDeadEyeXDrive,
            AutonConstants.kIDeadEyeXDrive,
            AutonConstants.kDDeadEyeXDrive,
            new Constraints(
                AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));

    addCommands(
        new SequentialCommandGroup(
            new ResetGyroCommand(driveSubsystem),
            new ParallelCommandGroup(
                new setAngleOffsetCommand(driveSubsystem, 0.0),
                new ZeroElbowCommand(elbowSubsystem),
                new SetFullCommand(magazineSubsystem),
                new SetHasPieceCommand(intakeSubsystem)),
            new PrepShooterCommand(
                superStructure, robotStateSubsystem, AutonConstants.Setpoints.NAS3),
            firstPath,
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
            new ParallelCommandGroup(
                secondPath,
                new ToDisruptCommand(
                    robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)),
            new TurnToAngleCommand(driveSubsystem, Rotation2d.fromDegrees(90)),
            thirdPath,
            new ParallelCommandGroup(
                shootPath,
                new SequentialCommandGroup(
                    new AutoWaitNoteStagedCommand(robotStateSubsystem),
                    new PrepShooterCommand(superStructure, robotStateSubsystem, shootPose))),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    shootPath.generateTrajectory();

    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && alliance == robotStateSubsystem.getAllianceColor();
  }
}
