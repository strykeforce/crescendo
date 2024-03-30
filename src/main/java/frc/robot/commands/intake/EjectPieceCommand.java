package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class EjectPieceCommand extends Command {
  private IntakeSubsystem intakeSubsystem;

  public EjectPieceCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.toEjecting();
  }

  @Override
  public boolean isFinished() {
    return !intakeSubsystem.isBeamBroken();
  }
}
