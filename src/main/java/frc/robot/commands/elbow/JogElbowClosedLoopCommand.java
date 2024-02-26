package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class JogElbowClosedLoopCommand extends Command {
  private ElbowSubsystem elbowSubsystem;
  private double jogRots;

  public JogElbowClosedLoopCommand(double jog, ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
    this.jogRots = jog;

    addRequirements(elbowSubsystem);
  }

  @Override
  public void execute() {
    elbowSubsystem.setPosition(elbowSubsystem.getPosition() + jogRots);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
