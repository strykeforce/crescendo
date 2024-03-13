package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ShooterConstants;
import frc.robot.standards.*;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {

  // Private Variables
  private ShooterIO io;
  private ShooterStates curState = ShooterStates.IDLE;
  private double leftSetpoint = 0.0;
  double rightSetpoint = 0.0;
  private boolean pctOut = false;

  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);
  Timer timer = new Timer();

  // Constructor
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return inputs.velocityLeft;
  }

  @Override
  public boolean atSpeed() {
    return pctOut
        || ((Math.abs(leftSetpoint - inputs.velocityLeft) < ShooterConstants.kCloseEnough
            && Math.abs(rightSetpoint - inputs.velocityRight) < ShooterConstants.kCloseEnough));
  }

  @Override
  public void setSpeed(double speed) {
    pctOut = false;
    leftSetpoint = speed;
    rightSetpoint = speed; // FIXME: add inversion where appropriate
    io.setSpeed(speed);
    timer.stop();
    timer.reset();
    timer.start();
  }

  public void setLeftSpeed(double speed) {
    pctOut = false;
    leftSetpoint = speed;
    io.setLeftSpeed(speed);
  }

  public void setRightSpeed(double speed) {
    pctOut = false;
    rightSetpoint = speed;
    io.setRightSpeed(speed);
  }

  public ShooterStates getState() {
    return curState;
  }

  public void setState(ShooterStates state) {
    curState = state;
  }

  public void toEmptying() {}

  // Helper Methods
  public void setPercent(double pct) {
    pctOut = true;
    io.setPct(pct);
  }

  public void enableFwdLimitSwitch(boolean enabled) {
    io.enableFwdLimitSwitch(enabled);
  }

  // Periodic
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Shooter", inputs);

    switch (curState) {
      case SHOOT:
        break;

      case SPEEDUP:
        if (atSpeed()) {
          curState = ShooterStates.SHOOT;
        }
        break;

      case IDLE:
        break;
      case PODIUM:
        break;
      case EMPTYING:
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
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  // State Enum
  public enum ShooterStates {
    SHOOT,
    SPEEDUP,
    IDLE,
    PODIUM,
    EMPTYING
  }
}
