package frc.robot.subsystems.auto;

public interface AutoCommandInterface {
  public void generateTrajectory();

  public boolean hasGenerated();

  public void schedule();
}
