package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem.ElbowStates;

public class ZeroRecoveryElbowCommand extends Command {
  private ElbowSubsystem elbowSubsystem;

  public ZeroRecoveryElbowCommand(ElbowSubsystem elbowSubsystem) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    elbowSubsystem.zeroRecovery();
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.getState() == ElbowStates.ZEROED;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
