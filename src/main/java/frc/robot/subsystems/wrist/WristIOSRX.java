package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.WristConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.AfterHealthCheck;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;

public class WristIOSRX implements WristIO, Checkable {
  private Logger logger;

  @HealthCheck
  @Position(
      percentOutput = {-0.1, 0.1},
      encoderChange = 2000)
  private TalonSRX wrist;

  public WristIOSRX() {

    logger = LoggerFactory.getLogger(this.getClass());
    wrist = new TalonSRX(WristConstants.kWristTalonSrxId);

    wrist.configFactoryDefault();
    wrist.configAllSettings(WristConstants.getSrxConfiguration());
    wrist.enableCurrentLimit(true);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist.enableVoltageCompensation(true);
  }

  @Override
  public String getName() {
    return "Wrist";
  }

  @Override
  public void zero() {
    double absolute = wrist.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double offset = absolute - RobotConstants.kWristZero;
    wrist.setSelectedSensorPosition(offset);

    logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absolute, RobotConstants.kWristZero, offset);
  }

  @Override
  public void setPosition(double position) {
    wrist.set(TalonSRXControlMode.MotionMagic, position);
  }

  @Override
  public void setPct(double percentOutput) {
    wrist.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.position = wrist.getSelectedSensorPosition();
    inputs.isRevLimitSwitch = wrist.isRevLimitSwitchClosed() == 1;
    inputs.isFwdLimitSwitchClosed = wrist.isFwdLimitSwitchClosed() == 1;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(wrist);
  }

  @BeforeHealthCheck
  @AfterHealthCheck
  public boolean goToZero() {
    setPosition(0.0);
    return Math.abs(wrist.getSelectedSensorPosition()) <= WristConstants.kCloseEnoughTicks;
  }
}
