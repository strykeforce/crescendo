package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ForkOpenLoopCommand extends InstantCommand {
  private ClimbSubsystem climbSubsystem;
  private double percent;

  public ForkOpenLoopCommand(ClimbSubsystem climbSubsystem, double percent) {
    this.climbSubsystem = climbSubsystem;
    this.percent = percent;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.setForkPercent(percent);
  }
}
