package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ClimbIOFX implements ClimbIO {
  private Logger logger;
  private TalonFX leftClimb;
  private TalonFX rightClimb;

  private double leftSetpoint;
  private double rightSetpoint;

  TalonFXConfigurator leftConfig;
  TalonFXConfigurator rightConfig;

  MotionMagicVoltage leftPosRequest = new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
  MotionMagicVoltage rightPosRequest = new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
  DutyCycleOut leftPctRequest = new DutyCycleOut(0.0);
  DutyCycleOut rightPctRequest = new DutyCycleOut(0.0);

  StatusSignal<Double> leftPos;
  StatusSignal<Double> rightPos;

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
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPosRots = leftPos.refresh().getValue();
    inputs.rightPosRots = rightPos.refresh().getValue();
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
  public void setCurrentLimit(CurrentLimitsConfigs config) {
    leftConfig.apply(config);
    rightConfig.apply(config);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(leftClimb, true);
    telemetryService.register(rightClimb, true);
  }
}