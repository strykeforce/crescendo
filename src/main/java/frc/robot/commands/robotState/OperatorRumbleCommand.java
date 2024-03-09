package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class OperatorRumbleCommand extends Command {

  RobotStateSubsystem robotStateSubsystem;
  Timer timer;
  XboxController xboxController;
  boolean hasStartedTimer = false;

  public OperatorRumbleCommand(
      RobotStateSubsystem robotStateSubsystem, XboxController xboxController) {
    this.xboxController = xboxController;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    xboxController.setRumble(RumbleType.kBothRumble, 1);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    xboxController.setRumble(RumbleType.kBothRumble, 0);
  }
}
