package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class SetElbowHasZeroedCommand extends InstantCommand {
  ElbowSubsystem elbowSubsystem;
  boolean val;

  public SetElbowHasZeroedCommand(ElbowSubsystem elbowSubsystem, boolean val) {
    this.elbowSubsystem = elbowSubsystem;
    this.val = val;
  }

  @Override
  public void initialize() {
    elbowSubsystem.setElbowZero(val);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
