package frc.robot.subsystems.elbow;

import frc.robot.constants.ElbowConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final ElbowIO io;
  // private final ElbowEncoderIO encoderIo;
  private final ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);
  private double setpoint = 0;
  private ElbowStates curState = ElbowStates.IDLE;

  public ElbowSubsystem(ElbowIO io, ElbowEncoderIO encoderIO) {
    this.io = io;
    // this.encoderIo = encoderIo;

    zero();
  }

  public void setPosition(double position) {
    io.setPosition(position);
    setpoint = position;
    curState = ElbowStates.MOVING;

    logger.info("Elbow moving to {} rotations", setpoint);
  }

  public void setPct(double pct) {
    io.setPct(pct);
    logger.info("Elbow open loop moving at {}", pct);
  }

  public double getPosition() {
    return inputs.positionRots;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public ElbowStates getState() {
    return curState;
  }

  public void setState(ElbowStates state) {
    curState = state;
  }

  public boolean isFinished() {
    return Math.abs(inputs.positionRots - setpoint) <= ElbowConstants.kCloseEnoughTicks;
  }

  public void zero() {
    io.zero();

    logger.info("Elbow zeroed");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Elbow", inputs);

    switch (curState) {
      case IDLE:
        break;

      case MOVING:
        if (isFinished()) {
          curState = ElbowStates.IDLE;
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

  public enum ElbowStates {
    IDLE,
    MOVING
  }
}
