package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleIsAutoCommand extends InstantCommand {
  RobotStateSubsystem robotStateSubsystem;

  public ToggleIsAutoCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAuto(!robotStateSubsystem.getIsAuto());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
