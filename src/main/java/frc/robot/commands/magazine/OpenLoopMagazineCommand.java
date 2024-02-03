package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.magazine.MagazineSubsystem;

public class OpenLoopMagazineCommand extends Command {
  private final MagazineSubsystem magazineSubsystem;
  private double percentOutput;

  public OpenLoopMagazineCommand(MagazineSubsystem magazineSubsystem, double percentOutput) {
    addRequirements(magazineSubsystem);

    this.magazineSubsystem = magazineSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    magazineSubsystem.setPercent(percentOutput);
  }
}
