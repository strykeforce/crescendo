package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class UnJamUpperNoteCommand extends Command {
  private MagazineSubsystem magazineSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public UnJamUpperNoteCommand(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.toUnJamTop();
    intakeSubsystem.toEjecting();
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isFwdBeamOpen();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      magazineSubsystem.setEmpty();
      robotStateSubsystem.toIntake();
    } else {
      magazineSubsystem.setFull();
      robotStateSubsystem.toStow();
    }
  }
}
