package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveStates;

public class SetDriveStateCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private DriveStates state;

  public SetDriveStateCommand(DriveSubsystem driveSubsystem, DriveStates state) {
    this.driveSubsystem = driveSubsystem;
    this.state = state;
  }

  @Override
  public void initialize() {
    driveSubsystem.setDriveState(state);
  }
}
