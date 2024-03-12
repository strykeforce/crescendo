package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.auto.AutoSwitch;

public class ToggleVirtualSwitchCommand extends InstantCommand {
  private AutoSwitch autoSwitch;

  public ToggleVirtualSwitchCommand(AutoSwitch autoSwitch) {
    this.autoSwitch = autoSwitch;
  }

  @Override
  public void initialize() {
    autoSwitch.toggleVirtualSwitch();
  }
}
