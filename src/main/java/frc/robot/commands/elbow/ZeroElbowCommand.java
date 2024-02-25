package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem.ElbowStates;

public class ZeroElbowCommand extends Command {
  ElbowSubsystem elbowSubsystem;

  public ZeroElbowCommand(ElbowSubsystem elbowSubsystem) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    elbowSubsystem.zero();
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.getState() != ElbowStates.ZEROING;
  }
}