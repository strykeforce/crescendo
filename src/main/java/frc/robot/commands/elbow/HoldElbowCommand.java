package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class HoldElbowCommand extends Command {
  private ElbowSubsystem elbowSubsystem;

  public HoldElbowCommand(ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
    addRequirements(elbowSubsystem);
  }

  @Override
  public void initialize() {
    elbowSubsystem.setPosition(elbowSubsystem.getSetpoint());
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.isFinished();
  }
}
