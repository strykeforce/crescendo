package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ToggleVisionUpdatesCommand extends InstantCommand {
  private VisionSubsystem visionSubsystem;
  private boolean on;

  public ToggleVisionUpdatesCommand(VisionSubsystem visionSubsystem, boolean on) {
    this.visionSubsystem = visionSubsystem;
    this.on = on;
  }

  @Override
  public void initialize() {
    visionSubsystem.setVisionUpdates(on);
  }
}
