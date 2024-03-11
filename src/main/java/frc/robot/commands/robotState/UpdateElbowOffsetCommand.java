package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import java.util.function.DoubleSupplier;

public class UpdateElbowOffsetCommand extends InstantCommand {
  RobotStateSubsystem robotStateSubsystem;
  DoubleSupplier offset;

  public UpdateElbowOffsetCommand(RobotStateSubsystem robotStateSubsystem, DoubleSupplier offset) {
    this.offset = offset;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setElbowOffsetPreferences(offset.getAsDouble());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
