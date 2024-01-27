package frc.robot.subsystems.magazine;

import frc.robot.constants.MagazineConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
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
  private WristSubsystem wristSubsystem;

  private double setpoint = inputs.velocity;

  // private int beamBroken = 0;
  // private int beamOpen = 0;

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

  public void setState(MagazineStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  public void setWristSubsystem(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
  }

  // Helper Methods
  public void toIntaking() {
    wristSubsystem.resetFwdBeamCounts();
    setSpeed(MagazineConstants.kIntakingSpeed);
    setState(MagazineStates.INTAKING);
  }

  public void toEmptying() {
    wristSubsystem.resetFwdBeamCounts();
    setSpeed(MagazineConstants.kEmptyingSpeed);
    setState(MagazineStates.EMPTYING);
  }

  public void preparePodium() {
    setSpeed(MagazineConstants.kPodiumPrepareSpeed);
    setState(MagazineStates.PREP_PODIUM);
  }

  public boolean hasPiece() {
    return curState == MagazineStates.FULL || curState == MagazineStates.EMPTYING;
  }

  // public boolean isBeamBroken() {
  //   if (inputs.isFwdLimitSwitchClosed) beamBroken++;
  //   else beamBroken = 0;

  //   return beamBroken > MagazineConstants.kMinBeamBreaks;
  // }

  // public boolean isBeamOpen() {
  //   if (!inputs.isFwdLimitSwitchClosed) beamOpen++;
  //   else beamOpen = 0;

  //   return beamOpen > MagazineConstants.kMinBeamBreaks;
  // }

  public boolean isNotePrepped() {
    // If the first edge of the note has been detected, set at edge one to be true
    if (!atEdgeOne && wristSubsystem.isRevBeamBroken()) {
      atEdgeOne = true;
    }

    // if the first edge has been detected, and the open space of the note has been
    // detected set
    // past first edge to be true
    if (atEdgeOne && !pastEdgeOne && wristSubsystem.isRevBeamBroken()) {
      pastEdgeOne = true;
    }

    // if the first edge, the open space has been detected, and the second edge of
    // the note has been
    // detected,
    // the note has been prepped.
    if (atEdgeOne && pastEdgeOne && wristSubsystem.isRevBeamBroken()) {
      return true;
    } else {
      return false;
    }
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
        if (wristSubsystem.isFwdBeamBroken()) {
          setSpeed(0.0);
          setState(MagazineStates.FULL);
        }
        break;
      case EMPTYING:
        break;
      case SPEEDUP:
        if (atShootSpeed()) {
          curState = MagazineStates.SHOOT;
        }
        break;
      case PREP_PODIUM:
        if (isNotePrepped()) {
          setSpeed(MagazineConstants.kShootSpeed);
          setState(MagazineStates.SPEEDUP);
          atEdgeOne = false;
          pastEdgeOne = false;
          wristSubsystem.resetRevBeamCounts();
        }
        break;
      case SHOOT:
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
    EMPTYING,
    SPEEDUP,
    PREP_PODIUM,
    SHOOT
  }
}
