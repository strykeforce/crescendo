package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class ClosedLoopElbowCommand extends Command {
    ElbowSubsystem elbowSubsystem;
    double setpoint;

    public ClosedLoopElbowCommand(ElbowSubsystem elbowSubsystem, double setpoint) {
        addRequirements(elbowSubsystem);
        this.elbowSubsystem = elbowSubsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        elbowSubsystem.setPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elbowSubsystem.isFinished();
    }
}
