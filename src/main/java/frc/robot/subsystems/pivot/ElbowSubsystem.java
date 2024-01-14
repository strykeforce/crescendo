package frc.robot.subsystems.pivot;

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
  private final ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);
  private double setpoint = 0;

  public ElbowSubsystem(ElbowIO io, ElbowEncoderIO encoderIO) {
    this.io = io;

    zero();
  }

  public void setPosition(double position) {
    io.setPosition(position);
    setpoint = position;
  }

  public double getPosition() {
    return inputs.positionTicks;
  }

  public boolean isFinished() {
    return Math.abs(inputs.positionTicks - setpoint) <= ElbowConstants.kCloseEnoughTicks;
  }

  public void zero() {
    io.zero();
    logger.info("Pivot zeroed");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // advLogger.processInputs("Pivot", inputs);

    // advLogger.recordOutput("Pivot/setpoint", setpoint);
  }

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
}
