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

  private double setpoint = inputs.position;

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

  public MagazineStates getState() {
    return curState;
  }

  public void setState(MagazineStates state) {
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

  public boolean hasPiece() {
    return curState == MagazineStates.FULL || curState == MagazineStates.EMPTYING;
  }

  // Periodic
  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (curState) {
      case EMPTY:
        break;
      case FULL:
        break;
      case INTAKING:
        if (io.getFwdLimitSwitch()) {
          logger.info("INTAKING -> FULL");
          curState = MagazineStates.FULL;
          io.setSpeed(0.0);
        }
        break;
      case EMPTYING:
        if (!io.getFwdLimitSwitch()) {
          logger.info("EMPTYING -> EMPTY");
          curState = MagazineStates.EMPTY;
          io.setSpeed(0.0);
        }
        break;
    }
  }
  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return null;
  }

  // State Enum
  public enum MagazineStates {
    EMPTY,
    FULL,
    INTAKING,
    EMPTYING
  }
}
