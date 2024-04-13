package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class SourceIntakeCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public SourceIntakeCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toSourceIntake();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.TO_HIGH_FEED
        || robotStateSubsystem.getState() != RobotStates.SOURCE_INTAKE
        || robotStateSubsystem.getState() != RobotStates.HIGH_FEED;
  }
}
