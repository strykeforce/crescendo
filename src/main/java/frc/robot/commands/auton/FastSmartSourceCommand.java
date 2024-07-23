package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.intake.SetHasPieceCommand;
import frc.robot.commands.magazine.SetFullCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.commands.robotState.PrepShooterCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import java.util.List;

public class FastSmartSourceCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private PathHandler pathHandler;
  private boolean hasGenerated = false;
  DriveAutonCommand firstPath;
  MiddleNoteDriveAutonCommand secondPath;
  List<Integer> preferences;
  String[][] pathNames;
  Double numPieces;
  Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private Pose2d shootPose;

  public FastSmartSourceCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      PathHandler pathHandler,
      DeadEyeSubsystem deadeye,
      LedSubsystem ledSubsystem,
      String firstPathName,
      String secondPathName,
      String[][] pathNames,
      List<Integer> preferences,
      Double numPieces,
      Pose2d shootPose) {
    addRequirements(
        driveSubsystem, superStructure, magazineSubsystem, intakeSubsystem, elbowSubsystem);
    firstPath = new DriveAutonCommand(driveSubsystem, firstPathName, true, true);
    secondPath =
        new MiddleNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            secondPathName,
            true,
            false);
    this.pathHandler = pathHandler;
    this.robotStateSubsystem = robotStateSubsystem;
    this.pathNames = pathNames;
    this.preferences = preferences;
    this.numPieces = numPieces;
    this.shootPose = shootPose;
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
            secondPath,
            new StartPathHandlerCommand(pathHandler)
            // new ToggleVisionUpdatesCommand(driveSubsystem)
            ));
  }

  public void generateTrajectory() {
    pathHandler.setPreference(preferences);
    pathHandler.setPaths(pathNames);
    pathHandler.setNumPieces(numPieces);
    pathHandler.generateTrajectory();
    pathHandler.setShotLoc(shootPose);
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && alliance == robotStateSubsystem.getAllianceColor();
  }
}