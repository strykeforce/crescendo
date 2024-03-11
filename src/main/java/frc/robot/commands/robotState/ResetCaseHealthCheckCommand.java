package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.strykeforce.healthcheck.internal.ResetCaseNum;

public class ResetCaseHealthCheckCommand extends InstantCommand {
  public ResetCaseHealthCheckCommand() {}

  @Override
  public void initialize() {
    new ResetCaseNum().resetCaseId();
  }
}
