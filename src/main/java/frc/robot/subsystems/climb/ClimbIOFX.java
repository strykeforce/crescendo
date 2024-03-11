package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;

public class ClimbIOFX implements ClimbIO, Checkable {
  private Logger logger;

  @HealthCheck
  @Position(
      percentOutput = {0.3, -0.3},
      encoderChange = 50)
  private TalonFX leftClimb;

  @HealthCheck
  @Follow(leader = ClimbConstants.kLeftClimbFxId)
  private TalonFX rightClimb;

  private double leftSetpoint;
  private double rightSetpoint;

  private TalonFXConfigurator leftConfig;
  private TalonFXConfigurator rightConfig;

  private MotionMagicVoltage leftPosRequest =
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
  private MotionMagicVoltage rightPosRequest =
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);

  private DutyCycleOut leftPctRequest = new DutyCycleOut(0.0);
  private DutyCycleOut rightPctRequest = new DutyCycleOut(0.0);

  private StatusSignal<Double> leftPos;
  private StatusSignal<Double> rightPos;
  private StatusSignal<Double> leftVel;
  private StatusSignal<Double> rightVel;

  public ClimbIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    leftClimb = new TalonFX(ClimbConstants.kLeftClimbFxId);
    rightClimb = new TalonFX(ClimbConstants.kRightClimbFxId);

    rightConfig = rightClimb.getConfigurator();
    leftConfig = leftClimb.getConfigurator();

    leftConfig.apply(new TalonFXConfiguration()); // factory default
    leftConfig.apply(ClimbConstants.getLeftConfig());

    rightConfig.apply(new TalonFXConfiguration());
    rightConfig.apply(ClimbConstants.getRightConfig());

    leftPos = leftClimb.getPosition().refresh();
    rightPos = rightClimb.getPosition().refresh();
    leftVel = leftClimb.getVelocity().refresh();
    rightVel = rightClimb.getVelocity().refresh();
  }

  @Override
  public String getName() {
    return "Climb";
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPosRots = leftPos.refresh().getValue();
    inputs.rightPosRots = rightPos.refresh().getValue();
    inputs.leftVelocity = leftVel.refresh().getValue();
    inputs.rightVelocity = rightVel.refresh().getValue();
  }

  @Override
  public void setPosition(double position) {
    leftClimb.setControl(leftPosRequest.withPosition(position));
    rightClimb.setControl(rightPosRequest.withPosition(position));
  }

  @Override
  public void setLeftPos(double position) {
    leftClimb.setControl(leftPosRequest.withPosition(position));
  }

  @Override
  public void setRightPos(double position) {
    rightClimb.setControl(rightPosRequest.withPosition(position));
  }

  @Override
  public void setPct(double percent) {
    leftClimb.setControl(leftPctRequest.withOutput(percent));
    rightClimb.setControl(rightPctRequest.withOutput(percent));
  }

  public void zero() {
    rightClimb.setPosition(0.0);
    leftClimb.setPosition(0.0);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enable) {
    logger.info("Set climb soft limits: {}", enable);
    leftConfig.apply(
        ClimbConstants.getLeftConfig()
            .SoftwareLimitSwitch
            .withForwardSoftLimitEnable(enable)
            .withReverseSoftLimitEnable(enable));
    rightConfig.apply(
        ClimbConstants.getRightConfig()
            .SoftwareLimitSwitch
            .withForwardSoftLimitEnable(enable)
            .withReverseSoftLimitEnable(enable));
  }

  @Override
  public void setCurrentLimit(CurrentLimitsConfigs config) {
    leftConfig.apply(config);
    rightConfig.apply(config);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(leftClimb, true);
    telemetryService.register(rightClimb, true);
  }

  @BeforeHealthCheck
  public boolean goToZero() {
    rightClimb.setControl(new Follower(ClimbConstants.kLeftClimbFxId, true));
    // setLeftPos(3);
    // // setPosition(1);
    // return Math.abs(leftPos.refresh().getValue() - 3) <= ClimbConstants.kCloseEnoughRots
    //     && Math.abs(rightPos.refresh().getValue() - 3) <= ClimbConstants.kCloseEnoughRots;
    return true;
  }
}
