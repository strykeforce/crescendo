package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class PostClimbStowCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public PostClimbStowCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ClimbSubsystem climbSubsystem) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem, climbSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.postClimbStow();
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return curState != RobotStates.TO_STOW;
  }
}
