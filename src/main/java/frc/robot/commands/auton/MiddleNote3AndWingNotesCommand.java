package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.IgnoreNotesCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.commands.robotState.PositionShootCommand;
import frc.robot.commands.robotState.SubWooferCommand;
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
  private DriveAutonCommand middleShoot3WingNote3;
  private DriveAutonCommand wingNote3MiddleInital1;
  private DriveAutonCommand wingNote3WingNote1;
  private DriveAutonCommand wingNote1Hunt;
  private PositionShootCommand midShootCommand;
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
    // middleNote3MiddleShoot3 =
    //     new DriveAutonCommand(driveSubsystem, "MiddleNote3_MiddleShoot3", true, false);
    // middleShoot3WingNote3 =
    //     new DriveAutonCommand(driveSubsystem, "MiddleShoot3_WingNote3", true, false);
    // wingNote3WingNote1 = new DriveAutonCommand(driveSubsystem, "WingNote3_WingNote1", true,
    // false);
    // wingNote1Hunt = new DriveAutonCommand(driveSubsystem, "WingNote1_Hunt", true, false);

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
                new WaitCommand(1.5),
                new IntakeCommand(
                    robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)))

        // new ParallelCommandGroup(
        //     middleNote3MiddleShoot3,
        //     new SequentialCommandGroup(
        //         new AutoWaitNoteStagedCommand(robotStateSubsystem),
        //         new PrepShooterCommand(
        //             superStructure,
        //             robotStateSubsystem,
        //             AutonConstants.Setpoints.MS3))),

        // new VisionShootCommand(robotStateSubsystem, superStructure, magazineSubsystem,
        //     intakeSubsystem),

        // middleShoot3WingNote3,
        // new AutoWaitNoteStagedCommand(robotStateSubsystem),
        // new VisionShootCommand(
        //     robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),

        // wingNote3WingNote1,
        // new AutoWaitNoteStagedCommand(robotStateSubsystem),
        // new VisionShootCommand(
        //     robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),

        // wingNote1Hunt,

        // new DeadeyeHuntCommand(deadeye, driveSubsystem, robotStateSubsystem, ledSubsystem),

        // new VisionShootCommand(
        //     robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem)
        );
  }

  public void generateTrajectory() {
    midInitMiddleNote3.generateTrajectory();
    // middleNote3MiddleShoot3.generateTrajectory();
    // middleShoot3WingNote3.generateTrajectory();
    // wingNote3MiddleInital1.generateTrajectory();
    // wingNote3WingNote1.generateTrajectory();
    // wingNote1Hunt.generateTrajectory();
    // midShootCommand.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
