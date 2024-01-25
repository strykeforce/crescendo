package frc.robot.subsystems.wrist;

import frc.robot.constants.WristConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class WristSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(WristSubsystem.class);
  private double setpoint = 0;
  private WristStates curState;

  public WristSubsystem(WristIO io) {
    this.io = io;

    zero();
  }

  public void setPosition(double position) {
    io.setPosition(position);
    setpoint = position;
    curState = WristStates.MOVING;

    logger.info("Wrist moving to {} ticks", setpoint);
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public WristStates getState() {
    return curState;
  }

  public void setState(WristStates state) {
    curState = state;
  }

  public boolean isFinished() {
    return Math.abs(inputs.position - setpoint) <= WristConstants.kCloseEnoughTicks;
  }

  public void zero() {
    io.zero();

    logger.info("Wrist zeroed");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Wrist", inputs);

    switch (curState) {
      case IDLE:
        break;

      case MOVING:
        if (isFinished()) {
          curState = WristStates.IDLE;
        }
        break;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public enum WristStates {
    IDLE,
    MOVING
  }
}
