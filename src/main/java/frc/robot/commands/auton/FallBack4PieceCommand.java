package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.SetHoloContKPCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.superStructure.SpinUpWheelsCommand;
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

public class FallBack4PieceCommand extends SequentialCommandGroup implements AutoCommandInterface {

  private DriveAutonCommand midInitWingNote3;
  private DriveAutonCommand wingNote3MidInit;
  private DriveAutonCommand midInitWingNote2;
  private DriveAutonCommand wingNote2MidInit;
  private DriveAutonCommand midInitWingNote1;
  private DriveAutonCommand wingNote1MidInit;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;
  private RobotStateSubsystem robotStateSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private DeadEyeSubsystem deadeye;

  public FallBack4PieceCommand(
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

    midInitWingNote3 =
        new DriveAutonCommand(driveSubsystem, "MiddleInitial1_WingNote3", true, true);
    wingNote3MidInit =
        new DriveAutonCommand(driveSubsystem, "WingNote3_MiddleInitial1", true, false);
    midInitWingNote2 =
        new DriveAutonCommand(driveSubsystem, "MiddleInitial1_WingNote2", true, false);

    wingNote2MidInit =
        new DriveAutonCommand(driveSubsystem, "WingNote2_MiddleInitial1", true, false);
    midInitWingNote1 =
        new DriveAutonCommand(driveSubsystem, "MiddleInitial1_WingNote1", true, false);
    wingNote1MidInit =
        new DriveAutonCommand(driveSubsystem, "WingNote1_MiddleInitial1", true, false);

    addCommands(
        new ResetGyroCommand(driveSubsystem),
        new ParallelCommandGroup(
            new SpinUpWheelsCommand(
                superStructure,
                SuperStructureConstants.kShooterSubwooferSetPoint,
                SuperStructureConstants.kShooterSubwooferSetPoint),
            new setAngleOffsetCommand(driveSubsystem, 0.0),
            new SetHoloContKPCommand(driveSubsystem, 1.0),
            new ZeroElbowCommand(elbowSubsystem)),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        midInitWingNote3,
        // new WaitCommand(0.05),
        // new AutoWaitNoteStagedCommand(robotStateSubsystem),

        wingNote3MidInit,
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        midInitWingNote2,
        wingNote2MidInit,
        // new WaitCommand(0.05),
        new AutoWaitNoteStagedCommand(robotStateSubsystem),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem),
        new ParallelCommandGroup(
            midInitWingNote1,
            wingNote1MidInit,
            // new WaitCommand(0.05),
            new SequentialCommandGroup(
                new AutoWaitNoteStagedCommand(robotStateSubsystem),
                new SpinUpWheelsCommand(
                    superStructure,
                    SuperStructureConstants.kShooterSubwooferSetPoint,
                    SuperStructureConstants.kShooterSubwooferSetPoint)),
            new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem)));
  }

  public void generateTrajectory() {
    midInitWingNote3.generateTrajectory();
    wingNote3MidInit.generateTrajectory();
    midInitWingNote2.generateTrajectory();
    wingNote2MidInit.generateTrajectory();
    midInitWingNote1.generateTrajectory();
    wingNote1MidInit.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
