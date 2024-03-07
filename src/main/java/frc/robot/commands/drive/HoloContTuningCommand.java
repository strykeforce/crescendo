package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class HoloContTuningCommand extends SequentialCommandGroup {
  private DriveAutonCommand testPath;
  private boolean hasGenerated = false;

  public HoloContTuningCommand(DriveSubsystem driveSubsystem) {
    testPath = new DriveAutonCommand(driveSubsystem, "5mTestPath", true, true);

    addCommands(
        new SequentialCommandGroup(
            new ToggleVisionUpdatesCommand(driveSubsystem),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.5), new OffsetOdomCommand(driveSubsystem, -0.5, 0)),
                testPath),
            new ToggleVisionUpdatesCommand(driveSubsystem)));
  }

  public void generateTrajectory() {
    testPath.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated;
  }
}
