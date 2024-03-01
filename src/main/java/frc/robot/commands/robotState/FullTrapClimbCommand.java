package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class FullTrapClimbCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public FullTrapClimbCommand(
      RobotStateSubsystem robotStateSubsystem,
      ClimbSubsystem climbSubsystem,
      SuperStructure superStructure) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(climbSubsystem, superStructure);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.climb(true);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() == RobotStates.TRAP;
  }
}
