package frc.robot.commands.pathHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.pathHandler.PathHandler.PathStates;

public class StartPathHandlerCommand extends Command {
  private PathHandler pathHandler;

  public StartPathHandlerCommand(PathHandler pathHandler) {
    this.pathHandler = pathHandler;
  }

  @Override
  public void initialize() {
    pathHandler.startHandling();
  }

  @Override
  public boolean isFinished() {
    return pathHandler.getState() == PathStates.DONE;
  }
}
