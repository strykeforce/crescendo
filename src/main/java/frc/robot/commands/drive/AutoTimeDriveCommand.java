package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class AutoTimeDriveCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double vX;
  private double vY;
  private double timeout;
  private Timer timer;

  public AutoTimeDriveCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      double vX,
      double vY,
      double timeout) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.vX = vX;
    this.vY = vY;
    this.timeout = timeout;
    this.timer = new Timer();

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      vX = -vX;
      driveSubsystem.setAutoDebugMsg("AutoTimeDriveCmd Flipping for red: " + vX + ", " + vY);
    }

    timer.reset();
    timer.start();
    driveSubsystem.move(vX, vY, 0.0, true);
    driveSubsystem.setAutoDebugMsg("Start AutoTimeDriveCmd: " + vX + ", " + vY);
  }

  @Override
  public void execute() {
    driveSubsystem.move(vX, vY, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0, 0, 0, true);
    driveSubsystem.setAutoDebugMsg("End AutoTimeDriveCmd: " + vX + ", " + vY);
  }
}
