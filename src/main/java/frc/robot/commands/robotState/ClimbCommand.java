package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class ClimbCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public ClimbCommand(
      RobotStateSubsystem robotStateSubsystem,
      ClimbSubsystem climbSubsystem,
      SuperStructure superStructure) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(superStructure, climbSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.climb(false);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.CLIMBING
        && robotStateSubsystem.getState() != RobotStates.FOLDING_OUT;
  }
}