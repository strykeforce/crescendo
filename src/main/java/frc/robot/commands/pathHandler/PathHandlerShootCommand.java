package frc.robot.commands.pathHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class PathHandlerShootCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private PathHandler pathHandler;
  private boolean shooting = false;

  public PathHandlerShootCommand(
      PathHandler pathHandler,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.pathHandler = pathHandler;
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.intakeHasNote() || robotStateSubsystem.magazineHasNote()) {
      pathHandler.startShot();
      shooting = true;
    }
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return !shooting || (curState != RobotStates.SHOOTING && curState != RobotStates.TO_SHOOT);
  }
}
