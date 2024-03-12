package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ToggleRatchetCommand extends InstantCommand {
  private ClimbSubsystem climbSubsystem;

  public ToggleRatchetCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.toggleClimbRatchet();
  }
}
