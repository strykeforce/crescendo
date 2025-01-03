package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class StowCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;

  public StowCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getState() == RobotStates.DEFENSE) {
      robotStateSubsystem.toDefenseStow();
    } else {
      robotStateSubsystem.toStow();
    }
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return curState != RobotStates.TO_STOW;
  }
}
