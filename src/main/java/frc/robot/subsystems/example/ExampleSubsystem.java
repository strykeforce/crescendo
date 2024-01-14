package frc.robot.subsystems.example;

import frc.robot.constants.ExampleConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ExampleSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  // Private Variables
  private final ExampleIO io;
  private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();
  private double setpoint = 0.0;
  private ExampleState curState = ExampleState.INIT;

  // Constructor
  public ExampleSubsystem(ExampleIO io) {
    this.io = io;
    zero();
  }

  // Getter/Setter Methods
  public ExampleState getState() {
    return curState;
  }

  @Override
  public double getPosition() {
    return inputs.position;
  }

  @Override
  public void setPosition(double position) {
    io.setPosition(position);
    setpoint = position;
  }

  // Helper Methods
  @Override
  public void zero() {
    io.zero();
    curState = ExampleState.ZEROED;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(setpoint - inputs.position) <= ExampleConstants.kCloseEnough;
  }

  // Periodic Function
  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(inputs);

    // State Machine
    switch (curState) {
      case INIT:
        break;
      case ZEROED:
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Example/curState", curState.ordinal());
    Logger.recordOutput("Example/setpoint", setpoint);
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {

    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  // State Enum
  public enum ExampleState {
    INIT,
    ZEROED
  }
}
