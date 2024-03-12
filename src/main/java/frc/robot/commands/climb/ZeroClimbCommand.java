package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbStates;

public class ZeroClimbCommand extends Command {
  private ClimbSubsystem climbSubsystem;

  public ZeroClimbCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.zeroAll();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.getState() == ClimbStates.ZEROED;
  }
}
