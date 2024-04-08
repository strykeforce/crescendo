package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbStates;

public class TrapClimbCommand extends Command {
  private ClimbSubsystem climbSubsystem;

  public TrapClimbCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.trapClimb();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.getState() != ClimbStates.CLIMBING;
  }
}
