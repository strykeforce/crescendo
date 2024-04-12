package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class EjectPieceCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public EjectPieceCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toEjecting();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.EJECTING;
  }
}
