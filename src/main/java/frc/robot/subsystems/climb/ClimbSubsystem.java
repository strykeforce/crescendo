package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
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
  private double leftForkSetpoint = 0.0;
  private double rightForkSetpoint = 0.0;
  private int climbZeroStableCounts = 0;
  private int leftForkZeroStableCounts = 0;
  private int rightForkZeroStableCounts = 0;
  private boolean hasLeftForkZeroed = false;
  private boolean hasRightForkZeroed = false;
  private boolean hasClimbZeroed = false;
  private int prepClimbRequestCount = 0;

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
    forkIO.setLeftPos(ClimbConstants.kLeftExtendPos);
    forkIO.setRightPos(ClimbConstants.kRightExtendPos);
    leftForkSetpoint = ClimbConstants.kLeftExtendPos;
    rightForkSetpoint = ClimbConstants.kRightExtendPos;
    // forkIO.setPct(0.3);
  }

  public void retractForks() {
    isForkExtended = false;
    forkIO.setLeftPos(ClimbConstants.kLeftRetractPos);
    forkIO.setRightPos(ClimbConstants.kRightRetractPos);
    leftForkSetpoint = ClimbConstants.kLeftRetractPos;
    rightForkSetpoint = ClimbConstants.kRightRetractPos;
    // forkTimer.reset();
    // forkTimer.start();
    // forkIO.setPct(-0.3);
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

  public void requestPrepClimb() {
    prepClimbRequestCount++;
  }

  public int getClimbRequestCount() {
    return prepClimbRequestCount;
  }

  public boolean hasClimbZeroed() {
    return hasClimbZeroed;
  }

  @Override
  public void zero() {
    zero(false);
  }

  public void zero(boolean proceedToClimb) {
    hasClimbZeroed = false;
    this.proceedToClimb = proceedToClimb;
    // climbIO.zero();
    logger.info("{} -> ZEROING", curState);
    curState = ClimbStates.ZEROING;
    enableRatchet(false);
    climbIO.setCurrentLimit(ClimbConstants.getZeroCurrentLimit());
    climbIO.setSoftLimitsEnabled(false);
    climbIO.setPct(ClimbConstants.kZeroPct);
  }

  public void zeroAll() {
    proceedToClimb = false;
    hasClimbZeroed = false;
    hasLeftForkZeroed = false;
    hasRightForkZeroed = false;
    climbZeroStableCounts = 0;
    logger.info("{} -> ZEROING_ALL", curState);
    curState = ClimbStates.ZEROING_ALL;
    
    forkIO.enableSoftLimits(false);
    forkIO.setPct(ClimbConstants.kZeroForkPct);
    
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
    // return Math.abs(leftForkSetpoint - forkInputs.leftPosTicks) <=
    // ClimbConstants.kCloseEnoughForks
    // && Math.abs(rightForkSetpoint - forkInputs.rightPosTicks)
    // <= ClimbConstants.kCloseEnoughForks;
    return true;
  }

  public void prepTrapClimb() {
    proceedToClimb = true;
    zero(true);
  }

  public void prepHighClimb() {
    logger.info("{} -> PREPPING", curState);

    extendForks();
    setPosition(ClimbConstants.kLeftClimbHighPrepPos);
    curState = ClimbStates.PREPPING;
  }

  public void trapClimb() {
    setPosition(ClimbConstants.kLeftClimbTrapPos);
    curState = ClimbStates.CLIMBING;
  }

  public void descend() {
    setPosition(ClimbConstants.kLeftClimbHighPrepPos);
    curState = ClimbStates.DESCENDING;
  }

  public void stow() {
    prepClimbRequestCount = 0;
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
          hasClimbZeroed = true;
          logger.info("{} -> ZEROED", curState);
          curState = ClimbStates.ZEROED;
        }
        break;
      case ZEROING_ALL:
        if (!hasLeftForkZeroed) {
          if (Math.abs(forkInputs.leftVelTicks) < ClimbConstants.kZeroForkMaxVel)
            leftForkZeroStableCounts++;
          else leftForkZeroStableCounts = 0;

          if (leftForkZeroStableCounts > ClimbConstants.kForkZeroStableCounts) {
            forkIO.zeroLeft();
            hasLeftForkZeroed = true;
            logger.info("Left Fork Zeroed");
            forkIO.setLeftPct(0.0);
          }
        }
        if (!hasRightForkZeroed) {
          if (Math.abs(forkInputs.rightVelTicks) < ClimbConstants.kZeroForkMaxVel)
            rightForkZeroStableCounts++;
          else rightForkZeroStableCounts = 0;

          if (rightForkZeroStableCounts > ClimbConstants.kForkZeroStableCounts) {
            forkIO.zeroRight();
            hasRightForkZeroed = true;
            forkIO.setRightPct(0.0);
            logger.info("Right Fork Zeroed");
          }
        }
        if (!hasClimbZeroed) {
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
            hasClimbZeroed = true;
            logger.info("Climb Zeroed");
          }
        }
        if (hasClimbZeroed && hasLeftForkZeroed && hasRightForkZeroed) {
          logger.info("{} -> ZEROED", curState);
          curState = ClimbStates.ZEROED;
          forkIO.enableSoftLimits(true);
          retractForks();
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
          if (prepClimbRequestCount > 1) {
            prepClimbRequestCount = 0;
            prepHighClimb();
          } else {
            prepClimbRequestCount = 0;
            logger.info("PREPPING -> PREPPED");
            curState = ClimbStates.PREPPED;
            forkIO.setPct(0.0);
          }
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
    org.littletonrobotics.junction.Logger.recordOutput("Climb State", curState);
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return Set.of(
        new Measure("Climb State", () -> curState.ordinal()),
        new Measure("hasClimbZeroed", () -> hasClimbZeroed ? 1.0 : 0.0),
        new Measure("hasLeftForkZeroed", () -> hasLeftForkZeroed ? 1.0 : 0.0),
        new Measure("hasRightForkZeroed", () -> hasRightForkZeroed ? 1.0 : 0.0));
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
    ZEROING_ALL,
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
