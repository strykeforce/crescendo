package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleAllianceColorCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public ToggleAllianceColorCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setAllianceColor(
        robotStateSubsystem.getAllianceColor() == Alliance.Blue ? Alliance.Red : Alliance.Blue);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
