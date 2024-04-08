package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import java.util.function.DoubleSupplier;

public class ClosedLoopElbowOffsetCommand extends Command {
  ElbowSubsystem elbowSubsystem;
  double setpoint;
  DoubleSupplier offset;

  public ClosedLoopElbowOffsetCommand(
      ElbowSubsystem elbowSubsystem, double setpoint, DoubleSupplier offset) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
    this.setpoint = setpoint;
    this.offset = offset;
  }

  @Override
  public void initialize() {
    elbowSubsystem.setPosition(setpoint + offset.getAsDouble(), true);
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.isFinished();
  }
}
