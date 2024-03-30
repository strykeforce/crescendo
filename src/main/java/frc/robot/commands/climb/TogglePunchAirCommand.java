package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbStates;

public class TogglePunchAirCommand extends Command {
  private ClimbSubsystem climbSubsystem;
  private boolean isStowing = false;

  public TogglePunchAirCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    if (climbSubsystem.getState() != ClimbStates.PUNCHING) {
      climbSubsystem.punchAir();
      isStowing = false;
    } else {
      climbSubsystem.stow();
      isStowing = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isStowing
        ? climbSubsystem.getState() == ClimbStates.STOWED
        : climbSubsystem.getState() == ClimbStates.PUNCHING;
  }
}
