package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.SetOmegaContKPCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.commands.robotState.EjectPieceCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.constants.DriveConstants;
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

public class SmartAmpIgnoreWingAutoCommand extends SequentialCommandGroup
    implements AutoCommandInterface {
  private PathHandler pathHandler;
  private boolean hasGenerated = false;
  private DriveAutonCommand firstPath;
  private MiddleNoteDriveAutonCommand secondPath;
  private DriveAutonCommand thirdPath;
  private List<Integer> preferences;
  private String[][] pathNames;
  private double numPieces;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private Pose2d shootPose;

  public SmartAmpIgnoreWingAutoCommand(
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
      // String thirdPathName,
      String[][] pathNames,
      List<Integer> preferences,
      double numPieces,
      Pose2d shootPose) {
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
            true,
            false);
    // thirdPath = new DriveAutonCommand(driveSubsystem, thirdPathName, true, false);
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
                new setAngleOffsetCommand(driveSubsystem, 90.0),
                new ZeroElbowCommand(elbowSubsystem)),
            new WaitCommand(0.01),
            firstPath,
            new ParallelCommandGroup(
                new SetOmegaContKPCommand(
                    driveSubsystem, DriveConstants.kPOmegaSpin, DriveConstants.kMaxAccelOmegaSpin),
                new SequentialCommandGroup(
                    new EjectPieceCommand(robotStateSubsystem, magazineSubsystem,
                    superStructure),
                    new IntakeCommand(
                        robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)),
                secondPath),
            new SetOmegaContKPCommand(
                driveSubsystem, DriveConstants.kPOmega, DriveConstants.kMaxAccelOmegaPath),
            new StartPathHandlerCommand(pathHandler)
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // thirdPath,
            // new TestDeadeyeCleanUpCommand(deadEyeSubsystem, driveSubsystem, robotStateSubsystem,
            // ledSubsystem),
            // new VisionShootCommand(robotStateSubsystem, superStructure, magazineSubsystem,
            // intakeSubsystem)
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
