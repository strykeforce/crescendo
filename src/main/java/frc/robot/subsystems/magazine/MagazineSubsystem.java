package frc.robot.subsystems.magazine;

import edu.wpi.first.wpilibj.Timer;
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
  private int fwdBeamBrokenCount = 0;
  private int fwdBeamOpenCount = 0;
  private int revBeamBrokenCount = 0;
  private int revBeamOpenCount = 0;

  private Timer releaseTimer = new Timer();

  // Podium Preparation Variables
  private boolean atEdgeOne = false;
  private boolean pastEdgeOne = false;

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

  public boolean atShootSpeed() {
    return Math.abs(inputs.velocity - setpoint) < MagazineConstants.kShootCloseEnough;
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

  private void setState(MagazineStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  // Helper Methods
  public void toIntaking(boolean isFast) {
    resetRevBeamCounts();
    io.enableRevLimitSwitch(true);
    setState(MagazineStates.INTAKING);
    setSpeed(isFast ? MagazineConstants.kFastIntakingSpeed : MagazineConstants.kIntakingSpeed);
  }

  public void toAmp() {
    io.enableFwdLimitSwitch(true);
    setSpeed(MagazineConstants.kReversingSpeed);
  }

  public void toEmptying() {
    resetRevBeamCounts();
    setSpeed(MagazineConstants.kEmptyingSpeed);
    io.enableLimitSwitches(false, false);
    setState(MagazineStates.EMPTYING);
  }

  public void toEmpty() {
    setState(MagazineStates.EMPTY);
    io.enableFwdLimitSwitch(false);
  }

  public void toEmptying(double speed) {
    resetRevBeamCounts();
    setSpeed(speed);
    io.enableLimitSwitches(false, false);
    setState(MagazineStates.EMPTYING);
  }

  public void toReleaseGamePiece() {
    releaseTimer.stop();
    releaseTimer.reset();
    releaseTimer.start();
    io.enableFwdLimitSwitch(false);
    setSpeed(MagazineConstants.kAmpReleaseSpeed);
    setState(MagazineStates.RELEASE);
  }

  public void setEmpty() {
    io.enableLimitSwitches(false, true);
    io.setPct(0.0);
    setState(MagazineStates.EMPTY);
  }

  public void setFull() {
    setState(MagazineStates.FULL);
  }

  public void preparePodium() {
    io.enableLimitSwitches(false, false);
    resetRevBeamCounts();
    setSpeed(MagazineConstants.kPodiumPrepareSpeed);
    setState(MagazineStates.PREP_PODIUM);
  }

  public void toPrepClimb() {
    io.enableFwdLimitSwitch(false);
    setSpeed(MagazineConstants.kReversingSpeed);
    setState(MagazineStates.REVERSING);
  }

  public void trap() {
    io.enableFwdLimitSwitch(false);
    setSpeed(MagazineConstants.kTrapReleaseSpeed);

    releaseTimer.reset();
    releaseTimer.start();

    setState(MagazineStates.TRAP);
  }

  public boolean hasPiece() {
    return curState == MagazineStates.FULL
        || curState == MagazineStates.EMPTYING
        || curState == MagazineStates.RELEASE;
  }

  public boolean isFwdBeamBroken() {
    if (inputs.isFwdLimitSwitchClosed) fwdBeamBrokenCount++;
    else fwdBeamBrokenCount = 0;

    return fwdBeamBrokenCount > MagazineConstants.kMinBeamBreaks;
  }

  public boolean isFwdBeamOpen() {
    if (!inputs.isFwdLimitSwitchClosed) fwdBeamOpenCount++;
    else fwdBeamOpenCount = 0;

    return fwdBeamOpenCount > MagazineConstants.kMinBeamBreaks;
  }

  public void resetFwdBeamCounts() {
    fwdBeamBrokenCount = 0;
    fwdBeamOpenCount = 0;
  }

  public boolean isRevBeamBroken() {
    if (inputs.isRevLimitSwitchClosed) revBeamBrokenCount++;
    else revBeamBrokenCount = 0;

    return revBeamBrokenCount > MagazineConstants.kMinBeamBreaks;
  }

  public boolean isRevBeamOpen() {
    if (!inputs.isRevLimitSwitchClosed) revBeamOpenCount++;
    else revBeamOpenCount = 0;

    return revBeamOpenCount > MagazineConstants.kMinBeamBreaks;
  }

  public void resetRevBeamCounts() {
    revBeamBrokenCount = 0;
    revBeamOpenCount = 0;
  }

  public void enableFwdLimitSwitch(boolean enabled) {
    io.enableFwdLimitSwitch(enabled);
  }

  public void enableRevLimitSwitch(boolean enabled) {
    io.enableRevLimitSwitch(enabled);
  }

  public void enableLimitSwitches(boolean enabled) {
    io.enableLimitSwitches(enabled, enabled);
  }

  public boolean isNotePrepped() {
    // If the first edge of the note has been detected, set at edge one to be true
    if (!atEdgeOne && isRevBeamBroken()) {
      atEdgeOne = true;
    }

    return atEdgeOne && isRevBeamOpen();
  }

  public void toEjecting() {
    resetRevBeamCounts();
    io.enableLimitSwitches(false, false);
    setSpeed(MagazineConstants.kEmptyingSpeed);
    setState(MagazineStates.EJECTING);
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
        if (isRevBeamBroken()) {
          // setSpeed(0.0);
          setState(MagazineStates.FULL);
        }
        break;
      case EMPTYING:
        io.enableRevLimitSwitch(false);
        break;
      case SPEEDUP:
        if (atShootSpeed()) {
          curState = MagazineStates.SHOOT;
        }
        break;
      case PREP_PODIUM:
        if (isNotePrepped()) {
          setSpeed(MagazineConstants.kPodiumShootSpeed);
          setState(MagazineStates.SPEEDUP);
          atEdgeOne = false;
          pastEdgeOne = false;
          resetRevBeamCounts();
        }
        break;
      case SHOOT:
        break;
      case RELEASE:
        if (releaseTimer.hasElapsed(MagazineConstants.kReleaseTime)) {
          setEmpty();
        }
        break;
      case REVERSING:
        if (isRevBeamOpen()) {
          setSpeed(0.0);
          setState(MagazineStates.FULL);
        }
        break;
      case TRAP:
        break;
      case EJECTING:
        io.enableLimitSwitches(false, false);
        break;
    }
    org.littletonrobotics.junction.Logger.recordOutput("States/Magazine State", curState);
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
    EMPTYING,
    SPEEDUP,
    PREP_PODIUM,
    SHOOT,
    RELEASE,
    TRAP,
    REVERSING,
    EJECTING
  }
}
