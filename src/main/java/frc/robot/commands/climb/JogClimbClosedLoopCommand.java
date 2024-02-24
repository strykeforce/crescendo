package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class JogClimbClosedLoopCommand extends Command {
  private ClimbSubsystem climbSubsystem;
  private double jogRots;

  public JogClimbClosedLoopCommand(double jog, ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.jogRots = jog;
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.setPosition(climbSubsystem.getSetpoint() + jogRots);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
