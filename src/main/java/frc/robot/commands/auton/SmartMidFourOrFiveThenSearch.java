package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoTimeDriveCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.commands.robotState.SubWooferCommand;
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

public class SmartMidFourOrFiveThenSearch extends SequentialCommandGroup
    implements AutoCommandInterface {
  private PathHandler pathHandler;
  private boolean hasGenerated = false;
  private MiddleNoteDriveAutonCommand firstPath;
  private WingNoteDriveAutonCommand secondPath;
  private DeadeyeHuntRotateCommand deadeyeHuntRotateCommand;
  private List<Integer> preferences;
  private String[][] pathNames;
  private double numPieces;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private Pose2d shootPose;

  public SmartMidFourOrFiveThenSearch(
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
      double numPieces,
      Pose2d shootPose) {
    addRequirements(
        driveSubsystem, superStructure, magazineSubsystem, intakeSubsystem, elbowSubsystem);
    firstPath =
        new MiddleNoteDriveAutonCommand(
            driveSubsystem, robotStateSubsystem, deadeye, ledSubsystem, firstPathName, true, true);
    secondPath =
        new WingNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            secondPathName,
            true,
            false);
    deadeyeHuntRotateCommand =
        new DeadeyeHuntRotateCommand(
            deadeye,
            driveSubsystem,
            robotStateSubsystem,
            ledSubsystem,
            AutonConstants.kHuntEndAngle);
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
                new setAngleOffsetCommand(driveSubsystem, -50.0),
                new ZeroElbowCommand(elbowSubsystem)),
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
            firstPath,
            new StartPathHandlerCommand(pathHandler),
            secondPath,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new AutoTimeDriveCommand(driveSubsystem, -0.5, 0.0, 0.5),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
            new TurnToAngleCommand(
                driveSubsystem, Rotation2d.fromRadians(AutonConstants.kHuntStartAngle)),
            deadeyeHuntRotateCommand,
            new AutoTimeDriveCommand(driveSubsystem, 0.5, 0.0, 0.5),
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)
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
    deadeyeHuntRotateCommand.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && alliance == robotStateSubsystem.getAllianceColor();
  }
}
