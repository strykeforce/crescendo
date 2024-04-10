package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class DriveDownLineCommand extends Command {
  private DeadEyeSubsystem deadeye;
  private DriveSubsystem driveSubsystem;
  private ProfiledPIDController deadeyeYDrive;

  public DriveDownLineCommand(DeadEyeSubsystem deadeye, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.deadeye = deadeye;

    deadeyeYDrive =
        new ProfiledPIDController(
            AutonConstants.kPDeadEyeYDrive,
            AutonConstants.kIDeadEyeYDrive,
            AutonConstants.kDDeadEyeYDrive,
            new Constraints(
                AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
  }

  @Override
  public void initialize() {
    double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
    driveSubsystem.recordYVel(ySpeed);
    driveSubsystem.move(AutonConstants.kForwardVel, ySpeed, 0.0, false);
  }
}
