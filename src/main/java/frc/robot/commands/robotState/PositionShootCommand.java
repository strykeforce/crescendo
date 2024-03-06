package frc.robot.commands.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class PositionShootCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  Pose2d shootPos;

  public PositionShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem, Pose2d shootPos) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.shootPos = shootPos;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.startShootKnownPos(shootPos);;
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return (curState != RobotStates.SHOOTING && curState != RobotStates.TO_SHOOT);
  }
}
