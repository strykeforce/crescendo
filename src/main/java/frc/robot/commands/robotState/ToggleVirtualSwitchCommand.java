package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoSwitch;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ToggleVirtualSwitchCommand extends Command {
  private AutoSwitch autoswitch;
  private static Logger logger;

  public ToggleVirtualSwitchCommand(AutoSwitch autoswitch) {
    this.autoswitch = autoswitch;
    logger = LoggerFactory.getLogger(ToggleVirtualSwitchCommand.class);
  }

  @Override
  public void initialize() {
    autoswitch.toggleVirtualSwitch();
    logger.info("toggledSwitch:command");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
