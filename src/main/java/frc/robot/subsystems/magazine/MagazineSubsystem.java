package frc.robot.subsystems.magazine;

import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MagazineSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  // Private Variables
  MagazineIO io;
  double setpoint = 0.0;
  MagazineStates curState = MagazineStates.EMPTY;

  // Constructor
  public MagazineSubsystem(MagazineIO io) {
    this.io = io;
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return io.getSpeed();
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(io.getSpeed() - setpoint) < MagazineConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    io.setSpeed(speed);
    ;
  }

  public MagazineStates getState() {
    return curState;
  }

  public void setState(MagazineStates state) {
    curState = state;
  }

  // Helper Methods

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case EMPTY:
        break;
      case FULL:
        break;
      case INTAKING:
        break;
      case EMPTYING:
        break;
    }
  }
  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }

  // State Enum
  public enum MagazineStates {
    EMPTY,
    FULL,
    INTAKING,
    EMPTYING
  }
}
