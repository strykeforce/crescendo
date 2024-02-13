package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class TunedShotCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;

  public TunedShotCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem) {
    addRequirements(superStructure, magazineSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toTune();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.TO_TUNE
        || robotStateSubsystem.getState() != RobotStates.SHOOTING;
  }
}
