package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;

public class ForkIOSRX implements ForkIO, Checkable {
  private Logger logger;

  @HealthCheck
  @Position(
      percentOutput = {0.3, -0.3},
      encoderChange = 450)
  private TalonSRX leftFork;

  @HealthCheck
  @Follow(leader = ClimbConstants.kLeftForkSRXId)
  private TalonSRX rightFork;

  @BeforeHealthCheck
  public boolean followTalons() {
    rightFork.follow(leftFork);
    return true;
  }

  public ForkIOSRX() {
    logger = LoggerFactory.getLogger(this.getClass());
    leftFork = new TalonSRX(ClimbConstants.kLeftForkSRXId);
    rightFork = new TalonSRX(ClimbConstants.kRightForkSRXId);

    leftFork.configFactoryDefault();
    leftFork.configAllSettings(ClimbConstants.getForkConfiguration());
    leftFork.enableCurrentLimit(true);
    leftFork.setNeutralMode(NeutralMode.Brake);
    leftFork.enableVoltageCompensation(true);

    rightFork.configFactoryDefault();
    rightFork.configAllSettings(ClimbConstants.getForkConfiguration());
    rightFork.enableCurrentLimit(true);
    rightFork.setNeutralMode(NeutralMode.Brake);
    rightFork.enableVoltageCompensation(true);
  }

  @Override
  public String getName() {
    return "Forks";
  }

  @Override
  public void updateInputs(ForkIOInputs inputs) {
    inputs.leftPosTicks = leftFork.getSelectedSensorPosition();
    inputs.rightPosTicks = rightFork.getSelectedSensorPosition();
    inputs.leftVelTicks = leftFork.getSelectedSensorVelocity();
    inputs.rightVelTicks = rightFork.getSelectedSensorVelocity();
  }

  @Override
  public void zeroLeft() {
    leftFork.setSelectedSensorPosition(0);
  }

  @Override
  public void zeroRight() {
    rightFork.setSelectedSensorPosition(0);
  }

  @Override
  public void setPct(double percent) {
    leftFork.set(ControlMode.PercentOutput, percent);
    rightFork.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setLeftPct(double percent) {
    leftFork.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setRightPct(double percent) {
    rightFork.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setPosition(double position) {
    leftFork.set(ControlMode.MotionMagic, position);
    rightFork.set(ControlMode.MotionMagic, position);
  }

  @Override
  public void setLeftPos(double position) {
    leftFork.set(ControlMode.MotionMagic, position);
  }

  @Override
  public void setRightPos(double position) {
    rightFork.set(ControlMode.MotionMagic, position);
  }

  @Override
  public void enableSoftLimits(boolean enable) {
    logger.info("Setting soft limits: {}", enable);
    leftFork.configForwardSoftLimitEnable(enable);
    leftFork.configReverseSoftLimitEnable(enable);
    rightFork.configForwardSoftLimitEnable(enable);
    rightFork.configReverseSoftLimitEnable(enable);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(leftFork);
    telemetryService.register(rightFork);
  }
}
