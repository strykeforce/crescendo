package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveSpeedSpinCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private XboxController xboxController;

  public DriveSpeedSpinCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.xboxController = xboxController;
  }

  @Override
  public void initialize() {
    driveSubsystem.move(0.5, 0, 1.5, true);
  }

  @Override
  public void execute() {
    driveSubsystem.move(0.5, 0, 1.5, true);
  }

  @Override
  public boolean isFinished() {
    if (!xboxController.getAButton()) {

      driveSubsystem.move(0, 0, 0, true);
      return true;
    }
    return false;
  }
}
