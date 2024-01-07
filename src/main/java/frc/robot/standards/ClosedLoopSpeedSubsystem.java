package frc.robot.standards;

public interface ClosedLoopSpeedSubsystem {

  public void setSpeed(double speed);

  public double getSpeed();

  public boolean atSpeed();
}
