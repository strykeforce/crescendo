package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class OffsetOdomCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private double dx;
  private double dy;

  public OffsetOdomCommand(DriveSubsystem driveSubsystem, double dx, double dy) {
    this.driveSubsystem = driveSubsystem;
    this.dx = dx;
    this.dy = dy;
  }

  @Override
  public void initialize() {
    Pose2d curr = driveSubsystem.getPoseMeters();
    driveSubsystem.resetOdometry(
        new Pose2d(curr.getX() + dx, curr.getY() + dy, curr.getRotation()));
  }
}
