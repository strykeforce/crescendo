package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class DeadeyeHuntCommand extends Command {
  private DeadEyeSubsystem deadeye;
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private ProfiledPIDController deadeyeYDrive;
  private PIDController deadeyeXDrive;
  private LedSubsystem ledSubsystem;
  private HuntState huntState;
  private int seenNote = 0;

  public DeadeyeHuntCommand(
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
                AutonConstants.kMaxAutonVelDeadeyeDrive, AutonConstants.kMaxAccelDeadeyeDrive));
    deadeyeXDrive =
        new PIDController(
            AutonConstants.kPDeadEyeXDrive,
            AutonConstants.kIDeadEyeXDrive,
            AutonConstants.kDDeadEyeXDrive);
    huntState = HuntState.SEARCHING;
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(120, 38, 109);

    driveSubsystem.setIsAligningShot(false);

    driveSubsystem.move(
        0.0,
        0.0,
        robotStateSubsystem.getAllianceColor() == Alliance.Blue
            ? AutonConstants.kDeadeyeHuntOmegaRadps
            : -AutonConstants.kDeadeyeHuntOmegaRadps,
        false);

    if (deadeye.getNumTargets() > 0) {
      seenNote++;
    }
  }

  @Override
  public void execute() {
    switch (huntState) {
      case SEARCHING:
        if (deadeye.getNumTargets() > 0) {
          seenNote++;
          if (seenNote >= AutonConstants.kFoundNoteLoopCounts) {
            huntState = HuntState.DRIVING;
          }
        } else {
          driveSubsystem.move(
              0.0,
              0.0,
              robotStateSubsystem.getAllianceColor() == Alliance.Blue
                  ? AutonConstants.kDeadeyeHuntOmegaRadps
                  : -AutonConstants.kDeadeyeHuntOmegaRadps,
              false);
          seenNote = 0;
        }
        break;
      case DRIVING:
        if (deadeye.getNumTargets() > 0) {
          double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
          double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
          driveSubsystem.recordYVel(ySpeed);
          driveSubsystem.move(AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.hasNote();
  }

  private enum HuntState {
    SEARCHING,
    DRIVING
  }
}
