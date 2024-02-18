package frc.robot.subsystems.climb;

import frc.robot.constants.ClimbConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private ClimbIO climbIO;
  private ClimbRatchetIO ratchetIO;
  private TrapBarIO trapIO;

  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();
  private final ClimbRatchetIOInputsAutoLogged ratchetInputs = new ClimbRatchetIOInputsAutoLogged();
  private final TrapBarIOInputsAutoLogged trapInputs = new TrapBarIOInputsAutoLogged();

  private Logger logger = LoggerFactory.getLogger(this.getClass());

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;
  private boolean isTrapBarExtended = false;
  private boolean isRatchetOn = false;

  public ClimbSubsystem(ClimbIO climbIO, ClimbRatchetIO ratchetIO, TrapBarIO trapIO) {
    this.climbIO = climbIO;
    this.ratchetIO = ratchetIO;
    this.trapIO = trapIO;
  }

  @Override
  public void setPosition(double position) {
    climbIO.setPosition(position);
    leftSetpoint = position;
    rightSetpoint = position;
    logger.info("Climb moving to {} rotations", position);
  }

  public void toggleTrapBar() {
    if (isTrapBarExtended) retractTrapBar();
    else extendTrapBar();
  }

  public void extendTrapBar() {
    trapIO.setPosition(RobotConstants.kLeftTrapBarRetract, RobotConstants.kRightTrapBarRetract);
    logger.info("Extending Trap Bar");
    isTrapBarExtended = false;
  }

  public void retractTrapBar() {
    trapIO.setPosition(RobotConstants.kLeftTrapBarExtend, RobotConstants.kRightTrapBarExtend);
    logger.info("Retracting Trap Bar");
    isTrapBarExtended = true;
  }

  public void toggleClimbRatchet() {
    if (isRatchetOn) enableRatchet(false);
    else enableRatchet(true);
  }

  public void enableRatchet(boolean enable) {
    if (enable) {
      ratchetIO.setPosition(RobotConstants.kLeftRatchetOn, RobotConstants.kRightRatchetOn);
      logger.info("Engaging Ratchet");
      isRatchetOn = true;
    } else {
      ratchetIO.setPosition(RobotConstants.kLeftRatchetOff, RobotConstants.kRightRatchetOff);
      logger.info("Disengaging Ratchet");
      isRatchetOn = false;
    }
  }

  @Override
  public double getPosition() {
    return climbInputs.leftPosRots;
  }

  public double getSetpoint() {
    return leftSetpoint;
  }

  @Override
  public void zero() {
    climbIO.zero();
    logger.info("Zeroed Climb");
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(leftSetpoint - climbInputs.leftPosRots) <= ClimbConstants.kCloseEnoughRots
        && Math.abs(rightSetpoint - climbInputs.rightPosRots) <= ClimbConstants.kCloseEnoughRots);
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    climbIO.registerWith(telemetryService);
  }
}
