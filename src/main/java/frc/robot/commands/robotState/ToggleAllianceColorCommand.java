package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ToggleAllianceColorCommand extends InstantCommand {
  private RobotContainer robotContainer;

  public ToggleAllianceColorCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    robotContainer.setAllianceColor(
        robotContainer.getAllianceColor() == Alliance.Blue ? Alliance.Red : Alliance.Blue);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
