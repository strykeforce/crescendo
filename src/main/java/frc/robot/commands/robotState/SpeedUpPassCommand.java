package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class SpeedUpPassCommand extends InstantCommand {
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;

  public SpeedUpPassCommand(
      RobotStateSubsystem robotStateSubsystem, SuperStructure superStructure) {
    addRequirements(superStructure);
    this.superStructure = superStructure;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.togglePassSpeedUp();
  }
}
