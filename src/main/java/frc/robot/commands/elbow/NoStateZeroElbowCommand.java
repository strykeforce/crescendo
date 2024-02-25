package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class NoStateZeroElbowCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;

  public NoStateZeroElbowCommand(ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    elbowSubsystem.zeroNoState();
  }
}
