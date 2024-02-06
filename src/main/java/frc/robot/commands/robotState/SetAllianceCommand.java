package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SetAllianceCommand extends Command {
  public Alliance alliance;
  public RobotContainer robotContainer;
  private static Logger logger = LoggerFactory.getLogger(SetAllianceCommand.class);

  public SetAllianceCommand(Alliance alliance, RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    robotContainer.setAllianceColor(alliance);
    logger.info("Manual Alliance color change: {}", alliance);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
