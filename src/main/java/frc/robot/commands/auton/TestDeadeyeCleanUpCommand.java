package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class TestDeadeyeCleanUpCommand extends Command {
  private DeadEyeSubsystem deadeye;
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private ProfiledPIDController deadeyeYDrive;
  private PIDController deadeyeXDrive;
  private Timer timeOut;
  private LedSubsystem ledSubsystem;

  public TestDeadeyeCleanUpCommand(
      DeadEyeSubsystem deadeye,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      LedSubsystem ledSubsystem) {
    addRequirements(driveSubsystem);
    this.deadeye = deadeye;
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.ledSubsystem = ledSubsystem;
    deadeyeYDrive =
        new ProfiledPIDController(
            AutonConstants.kPDeadEyeYDrive,
            AutonConstants.kIDeadEyeYDrive,
            AutonConstants.kDDeadEyeYDrive,
            new Constraints(
                AutonConstants.kMaxVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
    deadeyeXDrive =
        new PIDController(
            AutonConstants.kPDeadEyeXDrive,
            AutonConstants.kIDeadEyeXDrive,
            AutonConstants.kDDeadEyeXDrive);
    timeOut = new Timer();
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(120, 38, 109);
    timeOut.stop();
    timeOut.reset();
    timeOut.start();
    driveSubsystem.setIsAligningShot(false);
    if (deadeye.getNumTargets() > 0) {
      double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      driveSubsystem.move(AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
    }
  }

  @Override
  public void execute() {
    if (deadeye.getNumTargets() > 0) {
      double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      driveSubsystem.recordYVel(ySpeed);
      driveSubsystem.move(AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
    }
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.hasNote();
  }
}
