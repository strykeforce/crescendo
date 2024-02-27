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

import edu.wpi.first.wpilibj.Timer;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private ClimbIO climbIO;
  private ClimbRatchetIO ratchetIO;
  private TrapBarIO trapIO;
  private ForkIO forkIO;
  private boolean proceedToClimb = false;

  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();
  private final ClimbRatchetIOInputsAutoLogged ratchetInputs = new ClimbRatchetIOInputsAutoLogged();
  private final TrapBarIOInputsAutoLogged trapInputs = new TrapBarIOInputsAutoLogged();
  private final ForkIOInputsAutoLogged forkInputs = new ForkIOInputsAutoLogged();

  private Logger logger = LoggerFactory.getLogger(this.getClass());

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;
  private boolean isTrapBarExtended = false;
  private boolean isRatchetOn = false;
  private boolean isForkExtended = false;
  private int climbZeroStableCounts = 0;
  private double leftForkSetpoint = 0.0;
  private double rightForkSetpoint = 0.0;

  private ClimbStates curState = ClimbStates.IDLE;
  private Timer forkTimer = new Timer();

  public ClimbSubsystem(
      ClimbIO climbIO, ClimbRatchetIO ratchetIO, TrapBarIO trapIO, ForkIO forkIO) {
    this.climbIO = climbIO;
    this.ratchetIO = ratchetIO;
    this.trapIO = trapIO;
    this.forkIO = forkIO;

    enableRatchet(false);
    retractTrapBar();
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

  public void retractTrapBar() {
    trapIO.setPosition(RobotConstants.kLeftTrapBarRetract, RobotConstants.kRightTrapBarRetract);
    logger.info("Retracting Trap Bar");
    isTrapBarExtended = false;
  }

  public void extendTrapBar() {
    trapIO.setPosition(RobotConstants.kLeftTrapBarExtend, RobotConstants.kRightTrapBarExtend);
    logger.info("Extending Trap Bar");
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

  public void toggleForks() {
    if (isForkExtended) retractForks();
    else extendForks();
  }

  public void extendForks() {
    isForkExtended = true;
    // forkIO.setLeftPos(ClimbConstants.kLeftExtendPos);
    // forkIO.setRightPos(ClimbConstants.kRightExtendPos);
    // leftForkSetpoint = ClimbConstants.kLeftExtendPos;
    // rightForkSetpoint = ClimbConstants.kRightExtendPos;
    forkIO.setPct(0.3);
  }

  public void retractForks() {
    isForkExtended = false;
    // forkIO.setLeftPos(ClimbConstants.kLeftRetractPos);
    // forkIO.setRightPos(ClimbConstants.kRightRetractPos);
    // leftForkSetpoint = ClimbConstants.kLeftRetractPos;
    // rightForkSetpoint = ClimbConstants.kRightRetractPos;
    forkTimer.reset();
    forkTimer.start();
    forkIO.setPct(-0.3);
  }

  public void setForkPercent(double percent) {
    forkIO.setPct(percent);
  }

  @Override
  public double getPosition() {
    return climbInputs.leftPosRots;
  }

  public double getSetpoint() {
    return leftSetpoint;
  }

  public ClimbStates getState() {
    return curState;
  }

  @Override
  public void zero() {
    zero(false);
  }

  public void zero(boolean proceedToClimb) {
    this.proceedToClimb = proceedToClimb;
    // climbIO.zero();
    logger.info("{} -> ZEROING", curState);
    curState = ClimbStates.ZEROING;
    enableRatchet(false);
    climbIO.setCurrentLimit(ClimbConstants.getZeroCurrentLimit());
    climbIO.setSoftLimitsEnabled(false);
    climbIO.setPct(ClimbConstants.kZeroPct);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(leftSetpoint - climbInputs.leftPosRots) <= ClimbConstants.kCloseEnoughRots
        && Math.abs(rightSetpoint - climbInputs.rightPosRots) <= ClimbConstants.kCloseEnoughRots);
  }

  public boolean isForkFinished() {
    // return Math.abs(leftForkSetpoint - forkInputs.leftPosTicks) <= ClimbConstants.kCloseEnoughForks
    //     && Math.abs(rightForkSetpoint - forkInputs.rightPosTicks)
    //         <= ClimbConstants.kCloseEnoughForks;
    return true;
  }

  public void prepClimb() {
    proceedToClimb = true;
    zero(true);
  }

  public void trapClimb() {
    setPosition(ClimbConstants.kLeftClimbTrapPos);
    curState = ClimbStates.CLIMBING;
  }

  public void descend() {
    setPosition(ClimbConstants.kLeftClimbPrepPos);
    curState = ClimbStates.DESCENDING;
  }

  public void stow() {
    setPosition(ClimbConstants.kLeftStowPos);
    
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbInputs);
    ratchetIO.updateInputs(ratchetInputs);
    trapIO.updateInputs(trapInputs);
    forkIO.updateInputs(forkInputs);
    org.littletonrobotics.junction.Logger.processInputs("Climb Fx", climbInputs);
    org.littletonrobotics.junction.Logger.processInputs("Climb Ratchet", ratchetInputs);
    org.littletonrobotics.junction.Logger.processInputs("Trap Bar", trapInputs);
    org.littletonrobotics.junction.Logger.processInputs("Forks", forkInputs);

    switch (curState) {
      case IDLE:
        break;
      case ZEROING:
        if (Math.abs(climbInputs.leftVelocity) < ClimbConstants.kZeroSpeedThreshold
            && Math.abs(climbInputs.rightVelocity) < ClimbConstants.kZeroSpeedThreshold) {
          climbZeroStableCounts++;
        } else {
          climbZeroStableCounts = 0;
        }

        if (climbZeroStableCounts > ClimbConstants.kZeroStableCounts) {
          climbIO.zero();
          climbIO.setPct(0);
          climbIO.setCurrentLimit(ClimbConstants.getRunCurrentLimit());
          climbIO.setSoftLimitsEnabled(true);
          logger.info("{} -> ZEROED", curState);
          curState = ClimbStates.ZEROED;
        }
        break;
      case ZEROED:
        if (proceedToClimb) {
          logger.info("ZEROED -> PREPPING");
          curState = ClimbStates.PREPPING;
          extendForks();
          setPosition(ClimbConstants.kLeftClimbPrepPos);
        }
        break;
      case PREPPING:
        if (isFinished() && isForkFinished()) {
          logger.info("PREPPING -> PREPPED");
          curState = ClimbStates.PREPPED;
          forkIO.setPct(0.0);
        }
        break;
      case PREPPED:
        break;
      case CLIMBING:
        if (isFinished()) {
          logger.info("CLIMBING -> CLIMBED");
          curState = ClimbStates.CLIMBED;
        }
        break;
      case CLIMBED:
        break;
      case DESCENDING:
        if (isFinished()) {
          logger.info("DESCENDING -> DOWN");
          curState = ClimbStates.DOWN;
        }
        break;
      case DOWN:
        break;
      case STOWING:
        if (forkTimer.hasElapsed(0.5)) {
          forkIO.setPct(0.0);
        }
        if (isFinished()) {
          logger.info("STOWING -> STOWED");
          curState = ClimbStates.STOWED;
          forkIO.setPct(0.0);
        }
        break;
      case STOWED:
        break;
      default:
        break;
    }
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
    forkIO.registerWith(telemetryService);
  }

  public enum ClimbStates {
    IDLE,
    ZEROING,
    ZEROED,
    PREPPING,
    PREPPED,
    CLIMBING,
    CLIMBED,
    DESCENDING,
    DOWN,
    STOWING,
    STOWED
  }
}
