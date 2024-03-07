package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetHoloContKPCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private double kP;

  public SetHoloContKPCommand(DriveSubsystem driveSubsystem, double kP) {
    this.driveSubsystem = driveSubsystem;
    this.kP = kP;
  }

  @Override
  public void initialize() {
    driveSubsystem.setHolonomicControllerTranslationkP(kP);
  }
}
