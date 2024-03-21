package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class SetTrustElbowCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;
  private boolean ignoreElbow = false;

  public SetTrustElbowCommand(ElbowSubsystem elbowSubsystem, boolean val) {
    this.elbowSubsystem = elbowSubsystem;
    ignoreElbow = val;
  }

  @Override
  public void initialize() {
    elbowSubsystem.useElbowPos(ignoreElbow);
  }
}
