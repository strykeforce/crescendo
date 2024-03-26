package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ZeroWristCommand extends InstantCommand {
  private WristSubsystem wristSubsystem;

  public ZeroWristCommand(WristSubsystem wristSubsystem) {
    addRequirements(wristSubsystem);
    this.wristSubsystem = wristSubsystem;
  }

  @Override
  public void initialize() {
    wristSubsystem.zero();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
