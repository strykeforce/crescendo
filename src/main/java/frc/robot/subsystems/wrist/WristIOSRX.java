package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ElbowConstants;
import frc.robot.constants.WristConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class WristIOSRX implements WristIO {
  private Logger logger;
  private TalonSRX wrist;

  public WristIOSRX() {
    logger = LoggerFactory.getLogger(this.getClass());
    wrist = new TalonSRX(WristConstants.kWristTalonFxId);

    wrist.configFactoryDefault();
    wrist.configAllSettings(WristConstants.getSrxConfiguration());
  }

  @Override
  public void zero() {
    double absolute = wrist.getSensorCollection().getQuadraturePosition() & 0xFFF;
    double offset = absolute - WristConstants.kWristZeroTicks;
    wrist.setSelectedSensorPosition(offset);
    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}", absolute, ElbowConstants.kElbowZeroTicks, offset);
  }

  @Override
  public void setPosition(double position) {
    wrist.set(TalonSRXControlMode.MotionMagic, position);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.position = wrist.getSelectedSensorPosition();
    inputs.velocity = wrist.getSelectedSensorVelocity();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(wrist);
  }
}
