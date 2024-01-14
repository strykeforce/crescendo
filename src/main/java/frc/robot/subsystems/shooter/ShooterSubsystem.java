package frc.robot.subsystems.shooter;

import frc.robot.constants.ShooterConstants;
import frc.robot.standards.*;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {

  // Private Variables
  ShooterIO io;
  ShooterStates curState = ShooterStates.IDLE;
  private double setpoint = 0.0;

  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);

  // Constructor
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return io.getSpeed();
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(io.getSpeed() - setpoint) < ShooterConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    io.setSpeed(speed);
  }

  public ShooterStates getState() {
    return curState;
  }

  public void setState(ShooterStates state) {
    curState = state;
  }

  // Helper Methods

  // Periodic
  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (curState) {
      case SHOOT:
        break;

      case SPEEDUP:
        if (atSpeed()) {
          curState = ShooterStates.SHOOT;
        }
        break;

      case IDLE:
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
  public enum ShooterStates {
    SHOOT,
    SPEEDUP,
    IDLE
  }
}
