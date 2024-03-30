package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class TogglePunchAirCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private boolean isStowing = false;

  public TogglePunchAirCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    if (!robotStateSubsystem.getIsInDefense()) {
      robotStateSubsystem.toDefense();
      isStowing = false;
    } else {
      robotStateSubsystem.postClimbStow();
      isStowing = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isStowing
        ? robotStateSubsystem.getState() == RobotStates.STOW
            || robotStateSubsystem.getState() == RobotStates.INTAKING
        : true;
  }
}
