package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class IncrementRequestPrepClimbCommand extends InstantCommand {
  private ClimbSubsystem climbSubsystem;

  public IncrementRequestPrepClimbCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.requestPrepClimb();
  }
}
