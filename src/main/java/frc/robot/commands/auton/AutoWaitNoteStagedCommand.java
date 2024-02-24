package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class AutoWaitNoteStagedCommand extends Command {

  private RobotStateSubsystem robotStateSubsystem;

  public AutoWaitNoteStagedCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return !robotStateSubsystem.intakeHasNote()
        || (robotStateSubsystem.intakeHasNote() && robotStateSubsystem.magazineHasNote());
  }
}
