package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class DistanceShootCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  double distance;

  public DistanceShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      double distance) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.distance = distance;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.startShootDistance(distance);
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return (curState != RobotStates.SHOOTING || curState != RobotStates.TO_SHOOT);
  }
}
