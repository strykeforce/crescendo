package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class setAngleOffsetCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private double fixedRotation;
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);

  public setAngleOffsetCommand(DriveSubsystem driveSubsystem, double rotation) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.rotation = rotation;
  }

  public double fixedRotation(double rotation) {
    fixedRotation = driveSubsystem.apply(Rotation2d.fromDegrees(rotation)).getDegrees();
    logger.info("Previous rotation: {}, Fixed rotation: {}", rotation, fixedRotation);
    return fixedRotation;
  }

  @Override
  public void initialize() {
    driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(rotation));
  }
}
