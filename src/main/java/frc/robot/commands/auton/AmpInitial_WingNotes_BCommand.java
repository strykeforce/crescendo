package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.SetHoloContKPCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class AmpInitial_WingNotes_BCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private DriveAutonCommand firstPath;
  private DriveAutonCommand secondPath;
  private DriveAutonCommand thirdPath;
  private DriveAutonCommand fourthPath;
  private DriveAutonCommand fallbackPath;
  private DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private ElbowSubsystem elbowSubsystem;

  public AmpInitial_WingNotes_BCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, true, false);
    this.robotStateSubsystem = robotStateSubsystem;
    this.elbowSubsystem = elbowSubsystem;

    addCommands(
        new ResetGyroCommand(driveSubsystem),
        new ParallelCommandGroup(
            new setAngleOffsetCommand(driveSubsystem, 50.0),
            new SetHoloContKPCommand(driveSubsystem, 0.5),
            new ZeroElbowCommand(elbowSubsystem)),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        firstPath,
        new WaitCommand(0.1),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        new TurnToAngleCommand(driveSubsystem, Rotation2d.fromDegrees(-90.0)),
        secondPath,
        new WaitCommand(0.1),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        thirdPath,
        new WaitCommand(0.2),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
