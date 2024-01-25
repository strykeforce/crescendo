package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OpenLoopIntakeCommand extends InstantCommand {
  IntakeSubsystem intakeSubsystem;
  double pctOut = 0.0;

  public OpenLoopIntakeCommand(IntakeSubsystem intakeSubsystem, double pctOut) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.pctOut = pctOut;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setPercent(pctOut);
  }
}
