package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class ScoreTrapCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private boolean decend;

  public ScoreTrapCommand(
      RobotStateSubsystem robotStateSubsystem,
      ClimbSubsystem climbSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      boolean decendAfter) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.decend = decendAfter;

    addRequirements(superStructure, climbSubsystem, magazineSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.scoreTrap(decend);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.TRAP
        && robotStateSubsystem.getState() != RobotStates.FOLDING_IN;
  }
}
