package frc.robot.standards;

public interface ClosedLoopPosSubsystem {

  public void setPosition(double position);

  public double getPosition();

  public boolean isFinished();
}
