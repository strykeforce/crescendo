package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class TrapCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public TrapCommand(
      RobotStateSubsystem robotStateSubsystem,
      ClimbSubsystem climbSubsystem,
      SuperStructure superStructure) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(superStructure, climbSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toTrap();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.FOLDING_OUT
        && robotStateSubsystem.getState() != RobotStates.TO_TRAP;
  }
}
