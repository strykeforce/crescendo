package frc.robot.subsystems.elbow;

import frc.robot.constants.ElbowConstants;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final ElbowIO io;
  // private final ElbowEncoderIO encoderIo;
  private final ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);
  private double setpoint = 0;
  private ElbowStates curState = ElbowStates.IDLE;
  private int zeroStable = 0;
  private boolean hasZeroed = true;
  private int numRecoveryZeros = 0;
  private int recoveryZeroStableCounts = 0;
  private double prevRecoveryAbs = 0.0;

  private boolean isPrecise = false;

  public ElbowSubsystem(ElbowIO io) {
    this.io = io;
    // this.encoderIo = encoderIo;
    setpoint = inputs.positionRots;
    // io.zeroBlind();

    io.zero();
  }

  public void setPosition(double position) {
    setPosition(position, false);
  }

  public void setPosition(double position, boolean precise) {
    if (isPrecise != precise) {
      logger.info("isPrecise: {} -> {}", isPrecise, precise);
      if (precise) {
        io.setPreciseControl();
        io.setPosition(position, ElbowConstants.kPreciseSlot);
      } else {
        io.setPosition(position, ElbowConstants.kNormalSlot);
        io.setNormalControl();
      }
    } else {
      io.setPosition(
          position, isPrecise ? ElbowConstants.kPreciseSlot : ElbowConstants.kNormalSlot);
    }
    isPrecise = precise;

    if (setpoint != position) logger.info("Elbow moving to {} rotations", setpoint);

    setpoint = position;
    curState = ElbowStates.MOVING;
    if (position != setpoint) {
      logger.info("Elbow moving to {} rotations", setpoint);
    }
  }

  public void setPct(double pct) {
    io.setPct(pct);
    logger.info("Elbow open loop moving at {}", pct);
  }

  public double getPosition() {
    return inputs.positionRots;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public ElbowStates getState() {
    return curState;
  }

  public boolean getRevLimitSwitch() {
    return inputs.revLimitClosed;
  }

  public void setElbowZero(boolean val) {
    hasZeroed = val;
  }

  public void setState(ElbowStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  public boolean isFinished() {
    return Math.abs(inputs.positionRots - setpoint) <= ElbowConstants.kCloseEnoughRots;
  }

  public boolean hasZeroed() {
    return hasZeroed;
  }

  public void zeroNoState() {
    io.zeroBlind();
  }

  public void zero() {
    // io.zeroBlind();

    hasZeroed = false;
    // setState(ElbowStates.ZEROED);
    io.configHardwareLimit(ElbowConstants.getZeroLimitConfig());
    io.configMotionMagic(ElbowConstants.getZeroConfig());
    io.setPosition(ElbowConstants.kZeroPos, 0);
    setState(ElbowStates.ZEROING);
    setpoint = ElbowConstants.kZeroPos;
  }

  public void zeroRecovery() {
    io.zeroRecovery();
    setState(ElbowStates.RECOVERY_ZEROING);
    io.setPosition(0.0, 0);
    setpoint = 0.0;
    numRecoveryZeros = 1;

    logger.info("Recovery Zero Initial");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("Elbow", inputs);
    org.littletonrobotics.junction.Logger.recordOutput("Elbow Is Precise", isPrecise);

    switch (curState) {
      case IDLE:
        break;

      case MOVING:
        if (isFinished()) {
          curState = ElbowStates.IDLE;
        }
        break;
      case RECOVERY_ZEROING:
        // if (inputs.velocity <= ElbowConstants.kMinVelocityZeroing) zeroStable++;
        // else zeroStable = 0;
        // if (zeroStable > ElbowConstants.kMinStableZeroCounts) {
        //   setState(ElbowStates.ZEROED);
        //   io.zero();
        //   io.setPct(0.0);
        //   io.setCurrentLimit(ElbowConstants.getCurrentLimitConfig());
        // }
        if (Math.abs(inputs.absRots - prevRecoveryAbs) <= ElbowConstants.kCloseEnoughAbs)
          recoveryZeroStableCounts++;
        else recoveryZeroStableCounts = 0;

        if (recoveryZeroStableCounts > ElbowConstants.kStableCountsAbsEncoder) {
          io.zeroRecovery();
          logger.info("Zeroed");
          setState(ElbowStates.ZEROED);
        }

        break;
      case ZEROING:
        if (inputs.revLimitClosed) {
          io.configHardwareLimit(ElbowConstants.getRunLimitConfig());
          io.configMotionMagic(ElbowConstants.getRunConfig());

          // io.setHighResCANcoderPos();

          hasZeroed = true;
          logger.info("Zeroed");

          setPosition(SuperStructureConstants.kElbowSubwooferSetPoint);
          setState(ElbowStates.ZEROED);
        }
        break;
      case ZEROED:
        break;
    }
    org.littletonrobotics.junction.Logger.recordOutput("Elbow State", curState);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public enum ElbowStates {
    IDLE,
    MOVING,
    ZEROING,
    RECOVERY_ZEROING,
    ZEROED
  }
}
