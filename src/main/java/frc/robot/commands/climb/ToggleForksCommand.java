package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ToggleForksCommand extends Command {
  private ClimbSubsystem climbSubsystem;

  public ToggleForksCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.toggleForks();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isForkFinished();
  }
}
