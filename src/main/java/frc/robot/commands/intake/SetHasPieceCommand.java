package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetHasPieceCommand extends InstantCommand {
  private IntakeSubsystem intakeSubsystem;

  public SetHasPieceCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setHasNote();
  }
}
