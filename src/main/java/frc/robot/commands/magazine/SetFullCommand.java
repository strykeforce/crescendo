package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.magazine.MagazineSubsystem;

public class SetFullCommand extends InstantCommand {
  private MagazineSubsystem magazineSubsystem;

  public SetFullCommand(MagazineSubsystem magazineSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.setFull();
  }
}
