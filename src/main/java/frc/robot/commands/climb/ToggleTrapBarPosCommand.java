package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ToggleTrapBarPosCommand extends InstantCommand {
  private ClimbSubsystem climbSubsystem;

  public ToggleTrapBarPosCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.toggleTrapBar();
  }
}
