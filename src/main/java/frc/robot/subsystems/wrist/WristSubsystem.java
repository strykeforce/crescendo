package frc.robot.subsystems.wrist;

import frc.robot.constants.MagazineConstants;
import frc.robot.constants.SuperStructureConstants;
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
  private int revBeamBrokenCount = 0;
  private int revBeamOpenCount = 0;
  private int fwdBeamBrokenCount = 0;
  private int fwdBeamOpenCount = 0;
  private WristStates curState = WristStates.IDLE;

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

  public void setPct(double pct) {
    io.setPct(pct);
    logger.info("Wrist open loop moving at {}", pct);
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

  public boolean getIsRevLimitSwitch() {
    return inputs.isRevLimitSwitch;
  }

  public boolean isRevBeamBroken() {
    if (inputs.isRevLimitSwitch) revBeamBrokenCount++;
    else revBeamBrokenCount = 0;

    return revBeamBrokenCount > WristConstants.kMinBeamBreaks;
  }

  public boolean isRevBeamOpen() {
    if (!inputs.isRevLimitSwitch) revBeamOpenCount++;
    else revBeamOpenCount = 0;

    return revBeamOpenCount > WristConstants.kMinBeamBreaks;
  }

  public boolean isWristAtStow() {
    return Math.abs(getPosition() - SuperStructureConstants.kWristStowSetPoint)
        < MagazineConstants.kCloseEnough;
  }

  public void resetRevBeamCounts() {
    revBeamBrokenCount = 0;
    revBeamOpenCount = 0;
  }

  public boolean isFwdBeamBroken() {
    if (inputs.isFwdLimitSwitchClosed) fwdBeamBrokenCount++;
    else fwdBeamBrokenCount = 0;

    return fwdBeamBrokenCount > WristConstants.kMinBeamBreaks;
  }

  public boolean isFwdBeamOpen() {
    if (!inputs.isFwdLimitSwitchClosed) fwdBeamOpenCount++;
    else fwdBeamOpenCount = 0;

    return fwdBeamOpenCount > WristConstants.kMinBeamBreaks;
  }

  public void resetFwdBeamCounts() {
    fwdBeamBrokenCount = 0;
    fwdBeamOpenCount = 0;
  }

  public boolean isFinished() {
    return Math.abs(inputs.position - setpoint) <= WristConstants.kCloseEnoughTicks;
  }

  public void zero() {
    io.zero();

    logger.info("Wrist zeroed");
  }

  public void forceWristPos(double pos) {
    io.forceWristPos(pos);
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
    return Set.of(
        new Measure("state", () -> curState.ordinal()),
        new Measure("Fwd Beam Broken", () -> isFwdBeamBroken() ? 1.0 : 0.0),
        new Measure("Rev Beam Broken", () -> isRevBeamBroken() ? 1.0 : 0.0));
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
