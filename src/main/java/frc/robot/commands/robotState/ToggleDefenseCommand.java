package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class ToggleDefenseCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private RobotStates desiredState;

  public ToggleDefenseCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(superStructure, magazineSubsystem);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getState() == RobotStates.DEFENSE) {
      desiredState = RobotStates.INTAKING;
      robotStateSubsystem.toDefenseStow();
    } else {
      desiredState = RobotStates.DEFENSE;
      robotStateSubsystem.toDefense();
    }
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() == desiredState;
  }
}
