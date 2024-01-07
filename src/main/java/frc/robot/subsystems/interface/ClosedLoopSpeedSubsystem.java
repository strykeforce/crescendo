package frc.robot.subsystems.interface;

public interface ClosedLoopSpeedSubsystem {
    public double setpoint;

    public void setSpeed(double speed){}

    public double getSpeed(){}

    public boolean atSpeed(){}

    
}
