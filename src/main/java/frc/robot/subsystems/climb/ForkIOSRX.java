package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ForkIOSRX implements ForkIO {
  private Logger logger;
  private TalonSRX leftFork;
  private TalonSRX rightFork;

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
  public void updateInputs(ForkIOInputs inputs) {
    inputs.leftPosTicks = leftFork.getSelectedSensorPosition();
    inputs.rightPosTicks = rightFork.getSelectedSensorPosition();
  }

  @Override
  public void zero() {
    double leftAbs = leftFork.getSensorCollection().getPulseWidthPosition() & 0XFFF;
    double rightAbs = rightFork.getSensorCollection().getPulseWidthPosition() & 0xFFF;

    double leftOffset = leftAbs - ClimbConstants.kLeftForkZero;
    double rightOffset = rightAbs - ClimbConstants.kRightForkZero;

    leftFork.setSelectedSensorPosition(leftOffset);
    rightFork.setSelectedSensorPosition(rightOffset);

    logger.info(
        "Left Fork: Abs: {}, Zero Pos: {}, Offset: {}",
        leftAbs,
        ClimbConstants.kLeftForkZero,
        leftOffset);
    logger.info(
        "Right Fork: Abs: {}, Zero Pos: {}, Offset: {}",
        rightAbs,
        ClimbConstants.kRightForkZero,
        rightOffset);
  }

  @Override
  public void setPct(double percent) {
    leftFork.set(ControlMode.PercentOutput, percent);
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
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(leftFork);
    telemetryService.register(rightFork);
  }
}
