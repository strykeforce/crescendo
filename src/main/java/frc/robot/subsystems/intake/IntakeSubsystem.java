package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.standards.OpenLoopSubsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class IntakeSubsystem extends MeasurableSubsystem implements OpenLoopSubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs =
      new IntakeIOInputsAutoLogged(); // imports are not working properly waiting until vendordeps
  // configured
  private Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

  private IntakeState curState = IntakeState.NONE;

  private boolean beamBroken = false;
  private double beamBreakStableCounts = 0;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public IntakeState getState() {
    return curState;
  }

  private void setState(IntakeState state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  public boolean hasNote() {
    return (curState == IntakeState.HAS_PIECE) || curState == IntakeState.REVERSING;
  }

  public void toIntaking() {
    setPercent(IntakeConstants.kIntakePercentOutput);
    setState(IntakeState.INTAKING);
  }

  public void toReversing() {
    setPercent(IntakeConstants.kIntakeReversePercentOutput);
    setState(IntakeState.REVERSING);
  }

  public void toEjecting() {
    setPercent(IntakeConstants.kEjectPercent);
    setState(IntakeState.EJECTING);
  }

  public void stopIntaking() {
    setPercent(0.0);
    setState(IntakeState.NONE);
  }

  public void setFwdLimitSwitchSupplier(BooleanSupplier fwdLimitSupplier) {
    io.setFwdLimitSwitchSupplier(fwdLimitSupplier);
  }

  @Override
  public void setPercent(double pct) {
    io.setPct(pct);
  }

  // if the switch is closed, a stable count is incremented. if not, stable count is reset to zero.
  public boolean isBeamBroken() {
    if (inputs.isFwdLimitSwitchClosed) beamBreakStableCounts++;
    else beamBreakStableCounts = 0;

    beamBroken = (beamBreakStableCounts > IntakeConstants.kBeamBreakStableCounts);
    return beamBroken;
  }

  // intake state system
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Intake", inputs);

    switch (curState) {
      case HAS_PIECE:
        // has a gamepiece, does not disable intake
        break;
      case INTAKING:
        if (isBeamBroken()) {
          setState(IntakeState.HAS_PIECE);
        }
        break;
      case REVERSING:
        break;
      case EJECTING:
        break;
      default:
        break;
    }

    org.littletonrobotics.junction.Logger.recordOutput("Intake State", curState);
  }

  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> getState().ordinal()),
        new Measure("Beam Broken", () -> beamBroken ? 1.0 : 0.0));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public enum IntakeState {
    HAS_PIECE,
    INTAKING,
    EJECTING,
    REVERSING,
    NONE
  }
}
