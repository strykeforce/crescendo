package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;

public class GenerateTrajectories extends InstantCommand {
  private AutoCommandInterface drive;

  public GenerateTrajectories(AutoCommandInterface drive) {
    this.drive = drive;
  }

  @Override
  public void initialize() {
    drive.generateTrajectory();
  }
}
