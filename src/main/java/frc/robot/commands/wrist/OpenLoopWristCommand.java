package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

public class OpenLoopWristCommand extends Command {
  private final WristSubsystem wristSubsystem;
  private double percentOutput;

  public OpenLoopWristCommand(WristSubsystem wristSubsystem, double percentOutput) {
    addRequirements(wristSubsystem);

    this.wristSubsystem = wristSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    wristSubsystem.setPct(percentOutput);
  }
}
