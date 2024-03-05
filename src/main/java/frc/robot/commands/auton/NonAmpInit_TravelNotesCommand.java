package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class NonAmpInit_TravelNotesCommand extends SequentialCommandGroup {
  private DriveAutonCommand initialToNote3;
  private DriveAutonCommand note3ToNote4;
  private DriveAutonCommand note4ToNote5;
  private DriveAutonCommand note5ToNote3;
  private boolean hasGenerated = false;

  public NonAmpInit_TravelNotesCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    initialToNote3 =
        new DriveAutonCommand(driveSubsystem, "NonAmpInitial1_MiddleNote3", true, true);
    note3ToNote4 = new DriveAutonCommand(driveSubsystem, "MiddleNote3_MiddleNote4", true, false);
    note4ToNote5 = new DriveAutonCommand(driveSubsystem, "MiddleNote4_MiddleNote5", true, false);
    note5ToNote3 = new DriveAutonCommand(driveSubsystem, "MiddleNote5_MiddleNote3", true, false);

    addCommands(
        new SequentialCommandGroup(
            // new ToggleVisionUpdatesCommand(driveSubsystem),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)),
            initialToNote3, note3ToNote4, note4ToNote5, note5ToNote3));
  }

  public void generateTrajectory() {
    initialToNote3.generateTrajectory();
    note3ToNote4.generateTrajectory();
    note4ToNote5.generateTrajectory();
    note5ToNote3.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
