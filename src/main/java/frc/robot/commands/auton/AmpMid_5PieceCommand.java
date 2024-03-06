package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.SetHoloContKPCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
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

public class AmpMid_5PieceCommand extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand midInitWingNote3;
  DriveAutonCommand wingNote3WingNote2;
  DriveAutonCommand wingNote2WingNote1;
  DriveAutonCommand wingNote1MidNote1;
  DriveAutonCommand midNote1ShootPos;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private ElbowSubsystem elbowSubsystem;

  public AmpMid_5PieceCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elbowSubsystem = elbowSubsystem;

    midInitWingNote3 = new DriveAutonCommand(driveSubsystem, "MiddleStart_WingNote3", true, true);
    wingNote3WingNote2 =
        new DriveAutonCommand(driveSubsystem, "WingNote3_WingNote2_A", true, false);
    wingNote2WingNote1 =
        new DriveAutonCommand(driveSubsystem, "WingNote2_WingNote1_A", true, false);
    wingNote1MidNote1 = new DriveAutonCommand(driveSubsystem, "WingNote1_MiddleNote1", true, false);
    midNote1ShootPos =
        new DriveAutonCommand(driveSubsystem, "MiddleNote1_MiddleShoot", true, false);

    addCommands(
        new ResetGyroCommand(driveSubsystem),
        new ParallelCommandGroup(
            new setAngleOffsetCommand(driveSubsystem, 0.0),
            new SetHoloContKPCommand(driveSubsystem, 0.5),
            new ZeroElbowCommand(elbowSubsystem)),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        midInitWingNote3,
        new WaitCommand(0.1),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        wingNote3WingNote2,
        new WaitCommand(0.1),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        wingNote2WingNote1,
        new WaitCommand(0.2),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        wingNote1MidNote1,
        midNote1ShootPos,
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new VisionShootCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
  }

  public void generateTrajectory() {
    midInitWingNote3.generateTrajectory();
    wingNote3WingNote2.generateTrajectory();
    wingNote2WingNote1.generateTrajectory();
    wingNote1MidNote1.generateTrajectory();
    midNote1ShootPos.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
