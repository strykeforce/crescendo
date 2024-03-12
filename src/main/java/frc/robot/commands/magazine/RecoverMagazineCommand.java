package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.magazine.MagazineSubsystem;

public class RecoverMagazineCommand extends InstantCommand {
  MagazineSubsystem magazineSubsystem;

  public RecoverMagazineCommand(MagazineSubsystem magazineSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.toIntaking();
  }
}
