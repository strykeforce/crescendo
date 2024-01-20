package frc.robot.subsystems.shooter;

import frc.robot.constants.ShooterConstants;
import frc.robot.standards.*;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {

  // Private Variables
  ShooterIO io;
  ShooterStates curState = ShooterStates.IDLE;
  private double leftSetpoint = 0.0;
  double rightSetpoint = 0.0;

  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);

  // Constructor
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return inputs.velocityLeft;
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(inputs.velocityLeft - leftSetpoint) < ShooterConstants.kCloseEnough
        && Math.abs(inputs.velcoityRight - rightSetpoint) < ShooterConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    leftSetpoint = speed;
    rightSetpoint = speed;
    io.setSpeed(speed);
  }

  public void setLeftSpeed(double speed) {
    leftSetpoint = speed;
    io.setLeftSpeed(speed);
  }

  public void setRightSpeed(double speed) {
    rightSetpoint = speed;
    io.setRightSpeed(speed);
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

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  // State Enum
  public enum ShooterStates {
    SHOOT,
    SPEEDUP,
    IDLE
  }
}
