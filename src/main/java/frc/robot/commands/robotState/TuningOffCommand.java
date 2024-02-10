package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class TuningOffCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;
  MagazineSubsystem magazineSubsystem;
  IntakeSubsystem intakeSubsystem;

  public TuningOffCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(superStructure, magazineSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.enableLimitSwitches(true);
    magazineSubsystem.setSpeed(0.0);
    intakeSubsystem.stopIntaking();
    superStructure.stopShoot();
  }

  @Override
  public boolean isFinished() {
    return (superStructure.isFinished());
  }
}
