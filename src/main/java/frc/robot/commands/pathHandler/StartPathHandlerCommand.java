package frc.robot.commands.pathHandler;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pathHandler.PathHandler;

public class StartPathHandlerCommand extends InstantCommand {
  private PathHandler pathHandler;

  public StartPathHandlerCommand(PathHandler pathHandler) {
    this.pathHandler = pathHandler;
  }

  @Override
  public void initialize() {
    pathHandler.startHandling();
  }
}
