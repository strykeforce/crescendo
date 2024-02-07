package frc.robot.commands.auto;

public interface AutoCommandInterface {
    public void generateTrajectory();

    public boolean hasGenerated();

    public void schedule();
}
