package frc.robot.subsystems.magazine;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import frc.robot.constants.MagazineConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class MagazineIOFX implements MagazineIO {
  private TalonFX magazine;
  private Logger logger;

  private double setpoint;
  private final double absSensorIntial;

  TalonFXConfigurator configurator;
  private VelocityDutyCycle velocityRequest =
      new VelocityDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;
  StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  StatusSignal<ReverseLimitValue> revLimitSwitch;

  public MagazineIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    magazine = new TalonFX(MagazineConstants.kMagazineFalconID);
    absSensorIntial = magazine.getPosition().getValue();

    configurator = magazine.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(MagazineConstants.getMagazineConfig());

    currPosition = magazine.getPosition();
    currVelocity = magazine.getVelocity();
    fwdLimitSwitch = magazine.getForwardLimit();
    revLimitSwitch = magazine.getReverseLimit();
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.position = currPosition.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 1;
  }

  @Override
  public void setPct(double percentOutput) {
    magazine.set(percentOutput);
  }

  @Override
  public void setSpeed(double speed) {
    magazine.setControl(velocityRequest.withVelocity(speed));
  }

  @Override
  public double getSpeed() {
    return currVelocity.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(magazine, true);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    // TODO Auto-generated method stub
    MagazineIO.super.setSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }
}
