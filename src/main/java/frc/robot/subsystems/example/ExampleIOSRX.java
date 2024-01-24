package frc.robot.subsystems.example;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ExampleConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ExampleIOSRX implements ExampleIO {
  private TalonSRX talonSRX;
  private Logger logger;

  public ExampleIOSRX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonSRX = new TalonSRX(ExampleConstants.kExampleSrxId);

    // Configure
    talonSRX.configFactoryDefault();
    talonSRX.configAllSettings(ExampleConstants.getSRXConfig());
    talonSRX.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void zero() {
    double absolute = talonSRX.getSensorCollection().getQuadraturePosition() & 0xFFF;
    double offset = absolute - ExampleConstants.kZeroTicks;
    talonSRX.setSelectedSensorPosition(offset);
    logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absolute, ExampleConstants.kZeroTicks, offset);
  }

  @Override
  public void setPosition(double position) {
    talonSRX.set(ControlMode.MotionMagic, position);
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {
    inputs.position = talonSRX.getSelectedSensorPosition();
    inputs.velocity = talonSRX.getSelectedSensorVelocity();
    inputs.absPos = talonSRX.getSensorCollection().getQuadraturePosition() & 0xFFF;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonSRX);
  }
}
