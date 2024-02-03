package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elbow.ElbowSubsystem;

public class OpenLoopElbowCommand extends Command {
    private final ElbowSubsystem elbowSubsystem;
    private double percentOutput;

    public OpenLoopElbowCommand(ElbowSubsystem elbowSubsystem, double percentOutput) {
        addRequirements(elbowSubsystem);

        this.elbowSubsystem = elbowSubsystem;
        this.percentOutput = percentOutput;
    }

    @Override
    public void initialize() {
        elbowSubsystem.setPct(percentOutput);
    }
}
