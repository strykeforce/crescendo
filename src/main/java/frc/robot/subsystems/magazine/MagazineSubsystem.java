package frc.robot.subsystems.magazine;

import frc.robot.constants.MagazineConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MagazineSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  // Private Variables
  private MagazineIO io;
  private MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private MagazineStates curState = MagazineStates.EMPTY;
  private Logger logger = LoggerFactory.getLogger(MagazineSubsystem.class);

  private double setpoint = inputs.velocity;

  private int beamBroken = 0;
  private int beamOpen = 0;

  // Constructor
  public MagazineSubsystem(MagazineIO io) {
    this.io = io;
  }

  // Getter/Setter Methods
  @Override
  public double getSpeed() {
    return inputs.velocity;
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(inputs.velocity - setpoint) < MagazineConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    io.setSpeed(speed);
  }

  public void setPercent(double percentOutput) {
    io.setPct(percentOutput);
  }

  public MagazineStates getState() {
    return curState;
  }

  public void setState(MagazineStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  // Helper Methods
  public void toIntaking() {
    beamBroken = 0;
    setSpeed(MagazineConstants.kIntakingSpeed);
    setState(MagazineStates.INTAKING);
  }

  public void toEmptying() {
    beamOpen = 0;
    setSpeed(MagazineConstants.kEmptyingSpeed);
    setState(MagazineStates.EMPTYING);
  }

  public boolean hasPiece() {
    return curState == MagazineStates.FULL || curState == MagazineStates.EMPTYING;
  }

  public boolean isBeamBroken() {
    if (inputs.isFwdLimitSwitchClosed) beamBroken++;
    else beamBroken = 0;

    return beamBroken > MagazineConstants.kMinBeamBreaks;
  }

  public boolean isBeamOpen() {
    if (!inputs.isFwdLimitSwitchClosed) beamOpen++;
    else beamOpen = 0;

    return beamOpen > MagazineConstants.kMinBeamBreaks;
  }

  // Periodic
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Magazine", inputs);

    switch (curState) {
      case EMPTY:
        break;
      case FULL:
        break;
      case INTAKING:
        if (isBeamBroken()) {
          setSpeed(0.0);
          setState(MagazineStates.FULL);
        }
        break;
      case EMPTYING:
        if (isBeamOpen()) {
          setSpeed(0.0);
          setState(MagazineStates.EMPTY);
        }
        break;
    }
  }
  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
    super.registerWith(telemetryService);
  }

  // State Enum
  public enum MagazineStates {
    EMPTY,
    FULL,
    INTAKING,
    EMPTYING
  }
}
