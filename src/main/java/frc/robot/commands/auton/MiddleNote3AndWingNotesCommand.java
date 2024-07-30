package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.TurnUntilAngleCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.IgnoreNotesCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.commands.superStructure.SpinUpWheelsCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class MiddleNote3AndWingNotesCommand extends SequentialCommandGroup
    implements AutoCommandInterface {
  private MiddleNoteDriveAutonCommand midInitMiddleNote3;
  private DriveAutonCommand middleNote3MiddleShoot3;
  private WingNoteDriveAutonCommand middleShoot3WingNote3;
  private DriveAutonCommand wingNote3MidInit;
  private WingNoteDriveAutonCommand midInitWingNote1;
  private TurnUntilAngleCommand turnUntilAngleCommand;

  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private DeadEyeSubsystem deadeye;

  public MiddleNote3AndWingNotesCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      DeadEyeSubsystem deadeye,
      LedSubsystem ledSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.deadeye = deadeye;

    midInitMiddleNote3 =
        new MiddleNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            "MiddleInitial1_MiddleNote3",
            true,
            true);
    middleNote3MiddleShoot3 =
        new DriveAutonCommand(driveSubsystem, "MiddleNote3_MiddleShoot3", true, false);
    middleShoot3WingNote3 =
        new WingNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            "MiddleShoot3_WingNote3",
            true,
            false);
    wingNote3MidInit =
        new DriveAutonCommand(driveSubsystem, "WingNote3_MiddleInitial1", true, false);
    midInitWingNote1 =
        new WingNoteDriveAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            deadeye,
            ledSubsystem,
            "MiddleInitial1_WingNote1",
            true,
            false);

    turnUntilAngleCommand =
        new TurnUntilAngleCommand(
            driveSubsystem,
            robotStateSubsystem,
            AutonConstants.kDeadeyeHuntStartYawDegs,
            AutonConstants.kDeadeyeHuntOmegaRadps);

    addCommands(
        new ResetGyroCommand(driveSubsystem),
        new ParallelCommandGroup(
            new setAngleOffsetCommand(driveSubsystem, 0.0),
            new ZeroElbowCommand(elbowSubsystem),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem)),
        new ParallelCommandGroup(
            midInitMiddleNote3,
            new SequentialCommandGroup(
                new IgnoreNotesCommand(
                    robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
                new WaitCommand(2.0),
                new IntakeCommand(
                    robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem))),
        new ParallelCommandGroup(
            middleNote3MiddleShoot3,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new SpinUpWheelsCommand(
                superStructure,
                SuperStructureConstants.kShooterSubwooferSetPoint,
                SuperStructureConstants.kShooterSubwooferSetPoint)),
        new WaitCommand(0.02),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        middleShoot3WingNote3,
        new ParallelCommandGroup(
            wingNote3MidInit,
            new AutoWaitNoteStagedCommand(robotStateSubsystem),
            new SpinUpWheelsCommand(
                superStructure,
                SuperStructureConstants.kShooterSubwooferSetPoint,
                SuperStructureConstants.kShooterSubwooferSetPoint)),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        midInitWingNote1,
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        turnUntilAngleCommand,
        new DeadeyeHuntCommand(deadeye, driveSubsystem, robotStateSubsystem, ledSubsystem),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Backup
    // new DeadeyeHuntCommand(deadeye, driveSubsystem, robotStateSubsystem, ledSubsystem),
    // new AutoWaitNoteStagedCommand(robotStateSubsystem),
    // new VisionShootCommand(
    //     robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
  }

  public void generateTrajectory() {
    midInitMiddleNote3.generateTrajectory();
    middleNote3MiddleShoot3.generateTrajectory();
    middleShoot3WingNote3.generateTrajectory();
    wingNote3MidInit.generateTrajectory();
    midInitWingNote1.generateTrajectory();
    turnUntilAngleCommand.updateColor();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
