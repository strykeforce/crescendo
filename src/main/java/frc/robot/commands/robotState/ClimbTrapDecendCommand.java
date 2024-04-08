package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class ClimbTrapDecendCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private boolean climbAllowed;

  public ClimbTrapDecendCommand(
      RobotStateSubsystem robotStateSubsystem,
      ClimbSubsystem climbSubsystem,
      SuperStructure superStructure) {
    this.robotStateSubsystem = robotStateSubsystem;
    climbAllowed = true;
    addRequirements(climbSubsystem, superStructure);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getState() == RobotStates.CLIMB_PREPPED
        || robotStateSubsystem.getState() == RobotStates.POST_CLIMB) {
      robotStateSubsystem.climb(true, true);
    } else {
      climbAllowed = false;
    }
  }

  @Override
  public boolean isFinished() {
    return !climbAllowed || robotStateSubsystem.getState() == RobotStates.TRAP;
  }
}
