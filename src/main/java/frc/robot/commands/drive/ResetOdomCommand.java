package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdomCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private Pose2d pose;

  public ResetOdomCommand(DriveSubsystem driveSubsystem, Pose2d pose) {
    this.driveSubsystem = driveSubsystem;
    this.pose = pose;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(pose);
  }
}
