package frc.robot.subsystems.pivot;

import frc.robot.constants.PivotConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;

public class PivotSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(PivotSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();
  private double setpoint = 0;

  public PivotSubsystem(PivotIO io, PivotEncoderIO encoderIO) {
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
    return Math.abs(inputs.positionTicks - setpoint) <= PivotConstants.kCloseEnoughTicks;
  }

  public void zero() {
    
    logger.info("Pivot zeroed");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    advLogger.processInputs("Pivot", inputs);

    advLogger.recordOutput("Pivot/setpoint", setpoint);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }
}
