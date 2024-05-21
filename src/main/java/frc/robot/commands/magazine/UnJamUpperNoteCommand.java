package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.magazine.MagazineSubsystem;

public class UnJamUpperNoteCommand extends InstantCommand {
  private MagazineSubsystem magazineSubsystem;

  public UnJamUpperNoteCommand(MagazineSubsystem magazineSubsystem) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.toUnJamTop();
  }
}
