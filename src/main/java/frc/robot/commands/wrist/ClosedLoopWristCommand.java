package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ClosedLoopWristCommand extends Command {

  WristSubsystem wristSubsystem;
  double setPoint;

  public ClosedLoopWristCommand(WristSubsystem wristSubsystem, double setPoint) {
    addRequirements(wristSubsystem);
    this.wristSubsystem = wristSubsystem;
    this.setPoint = setPoint;
  }

  @Override
  public void initialize() {
    wristSubsystem.setPosition(setPoint);
  }

  @Override
  public boolean isFinished() {
    return wristSubsystem.isFinished();
  }
}
