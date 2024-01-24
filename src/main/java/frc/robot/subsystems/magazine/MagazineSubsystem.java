package frc.robot.subsystems.magazine;

import frc.robot.constants.MagazineConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MagazineSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  // Private Variables
  private MagazineIO io;
  private MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private MagazineStates curState = MagazineStates.EMPTY;
  private Logger logger = LoggerFactory.getLogger(MagazineSubsystem.class);

  private double setpoint = inputs.velocity;

  private boolean edgeOne;
  private boolean edgeTwo;

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

  // Helper Methods
  public void toIntaking() {
    setSpeed(MagazineConstants.kIntakingSpeed);
    setState(MagazineStates.INTAKING);
  }

  public void toEmptying() {
    setSpeed(MagazineConstants.kEmptyingSpeed);
    setState(MagazineStates.EMPTYING);
  }

  public void toShooting() {
    setSpeed(MagazineConstants.kShootSpeed);
    setState(MagazineStates.SHOOT);
  }

  public void preparePodium() {
    setSpeed(MagazineConstants.kPodiumPrepareSpeed);
    setState(MagazineStates.PREPARING_PODIUM);
  }

  public boolean hasPiece() {
    return curState == MagazineStates.FULL || curState == MagazineStates.EMPTYING;
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
        if (inputs.isFwdLimitSwitchClosed) {
          setSpeed(0.0);
          setState(MagazineStates.FULL);
        }
        break;
      case EMPTYING:
        if (!inputs.isFwdLimitSwitchClosed) {
          setSpeed(0.0);
          setState(MagazineStates.EMPTY);
        }
        break;
      case SPEEDUP:
        if (atShootSpeed()) {
          curState = MagazineStates.SHOOT;
        }
        break;
      case PREPARING_PODIUM:
        //if we have edge 2 beam break was triggered, go to the shooting speed (edge 2 meaning 2nd edge of the note)
        // if (inputs.isSecondFwdLimitSwitchClosed) {
        //  setSpeed(0.0);
        //  setState(MagazineStates.SPEEDUP);
        // }
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

  // State Enum
  public enum MagazineStates {
    EMPTY,
    FULL,
    INTAKING,
    EMPTYING,
    SPEEDUP,
    PREPARING_PODIUM,
    SHOOT
  }
}
