package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class DynamicZeroPosCommand extends SequentialCommandGroup {
  private ElbowSubsystem elbowSubsystem;

  public DynamicZeroPosCommand(ElbowSubsystem elbowSubsystem, double position) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
    addCommands(
        new ClosedLoopElbowCommand(elbowSubsystem, position),
        new WaitCommand(5),
        new NoStateZeroElbowCommand(elbowSubsystem),
        new ClosedLoopElbowCommand(elbowSubsystem, position));
  }
}
