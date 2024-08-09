package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;
import net.jafama.FastMath;

public class DeadeyeHuntRotateCommand extends Command implements AutoCommandInterface {
  private DeadEyeSubsystem deadeye;
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private ProfiledPIDController deadeyeYDrive;
  private PIDController deadeyeXDrive;
  private LedSubsystem ledSubsystem;
  private double turnTo;
  private double turnToInverted;
  private double deltaTurn;
  private boolean reachedEnd = false;
  private boolean generated = false;

  public DeadeyeHuntRotateCommand(
      DeadEyeSubsystem deadeye,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      LedSubsystem ledSubsystem,
      double turnTo) {
    addRequirements(driveSubsystem);
    this.deadeye = deadeye;
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.turnTo = turnTo;
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
  }

  @Override
  public void initialize() {
    driveSubsystem.setAutoDebugMsg("DeadEYE Hunt Initialized");
    deltaTurn =
        turnToInverted - getCorrectRange() >= 0
            ? AutonConstants.kHuntTurnSpeed
            : -AutonConstants.kHuntTurnSpeed;
    ledSubsystem.setColor(120, 38, 109);

    driveSubsystem.setIsAligningShot(false);
    if (deadeye.getNumTargets() > 0) {
      double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      driveSubsystem.move(AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
    } else if (deltaTurn > 0 && turnToInverted < getCorrectRange()
        || deltaTurn < 0 && turnToInverted > getCorrectRange()) {
      reachedEnd = true;
    } else {
      driveSubsystem.move(0.0, 0.0, deltaTurn, false);
    }
  }

  @Override
  public void execute() {
    if (deadeye.getNumTargets() > 0) {
      double ySpeed = deadeyeYDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      double xSpeed = deadeyeXDrive.calculate(deadeye.getDistanceToCamCenter(), 0.0);
      driveSubsystem.recordYVel(ySpeed);
      driveSubsystem.move(AutonConstants.kXSpeed / (xSpeed * xSpeed + 1), ySpeed, 0.0, false);
    } else if (deltaTurn > 0 && turnToInverted < getCorrectRange()
        || deltaTurn < 0 && turnToInverted > getCorrectRange()) {
      reachedEnd = true;
    } else {
      driveSubsystem.move(0.0, 0.0, deltaTurn, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setAutoDebugMsg("Finished Deadeye Hunt");
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.hasNote();
  }

  @Override
  public void generateTrajectory() {
    turnToInverted =
        robotStateSubsystem.getAllianceColor() == Alliance.Blue
            ? FastMath.normalizeZeroTwoPi(turnTo)
            : Math.PI - FastMath.normalizeZeroTwoPi(turnTo);

    generated = true;
  }

  @Override
  public boolean hasGenerated() {
    return generated;
  }

  public double getCorrectRange() {
    return robotStateSubsystem.getAllianceColor() == Alliance.Blue
        ? FastMath.normalizeZeroTwoPi(driveSubsystem.getGyroRotation2d().getRadians())
        : driveSubsystem.getGyroRotation2d().getRadians();
  }
}
