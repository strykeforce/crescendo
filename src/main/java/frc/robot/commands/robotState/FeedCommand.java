package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class FeedCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  boolean flag = false;

  public FeedCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    flag =
        !((robotStateSubsystem.intakeHasNote() && robotStateSubsystem.magazineHasNote()))
            && robotStateSubsystem.getIsAuto();
    if (!flag) robotStateSubsystem.toFixedFeeding();
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return flag || (curState != RobotStates.SHOOTING && curState != RobotStates.TO_FEED);
  }
}
