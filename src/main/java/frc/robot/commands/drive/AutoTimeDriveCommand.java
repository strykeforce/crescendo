package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoTimeDriveCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private double vX;
  private double vY;
  private double timeout;
  private Timer timer;

  public AutoTimeDriveCommand(DriveSubsystem driveSubsystem, double vX, double vY, double timeout) {
    this.driveSubsystem = driveSubsystem;
    this.vX = vX;
    this.vY = vY;
    this.timeout = timeout;
    this.timer = new Timer();

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveSubsystem.move(vX, vY, 0.0, true);
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
  }
}
