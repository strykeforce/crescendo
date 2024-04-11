package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.DistanceShootCommand;
import frc.robot.commands.robotState.ToDisruptCommand;
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
import java.util.List;

public class DisruptAutonCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private boolean hasGenerated = false;

  private MiddleNoteDriveAutonCommand firstPath;
  private DriveAutonCommand secondPath;
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
      String[][] pathNames,
      List<Integer> preferences,
      Double numPieces,
      Pose2d shootPose) {
    addRequirements(
        driveSubsystem, superStructure, magazineSubsystem, intakeSubsystem, elbowSubsystem);
    firstPath =
        new MiddleNoteDriveAutonCommand(
            driveSubsystem, robotStateSubsystem, deadeye, ledSubsystem, firstPathName, false, true);
    secondPath = new DriveAutonCommand(driveSubsystem, secondPathName, false, false);

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
                new ZeroElbowCommand(elbowSubsystem)),
            firstPath,
            new DistanceShootCommand(
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem,
                AutonConstants.kNAS3ToSpeakerDist),
            secondPath,
            new ToDisruptCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
            new DriveCenterLineCommand(
                driveSubsystem,
                robotStateSubsystem,
                deadeye,
                ledSubsystem,
                thirdPathName,
                false,
                false)));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();

    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && alliance == robotStateSubsystem.getAllianceColor();
  }
}
