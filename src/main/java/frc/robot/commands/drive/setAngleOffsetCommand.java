package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class setAngleOffsetCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private Rotation2d fixedRotation;

  public setAngleOffsetCommand(DriveSubsystem driveSubsystem, double rotation) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.rotation = rotation;
  }

  public Rotation2d fixedRotation() {
    fixedRotation = driveSubsystem.apply(Rotation2d.fromDegrees(rotation));
    return fixedRotation;
  }

  @Override
  public void initialize() {
    driveSubsystem.setGyroOffset(fixedRotation());
    ;
  }
}
