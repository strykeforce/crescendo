package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import frc.robot.constants.ShooterConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ShooterIOFX implements ShooterIO {
  private TalonFX shooter;
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

  public ShooterIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    shooter = new TalonFX(ShooterConstants.kShooterTalonID);
    absSensorIntial = shooter.getPosition().getValue();

    configurator = shooter.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(ShooterConstants.getShooterConfig());

    currPosition = shooter.getPosition();
    currVelocity = shooter.getVelocity();
    fwdLimitSwitch = shooter.getForwardLimit();
    revLimitSwitch = shooter.getReverseLimit();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.position = currPosition.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 1;
  }

  @Override
  public void setPct(double percentOutput) {
    shooter.set(percentOutput);
  }

  @Override
  public void setSpeed(double speed) {
    shooter.setControl(velocityRequest.withVelocity(speed));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shooter, true);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    // TODO Auto-generated method stub
    ShooterIO.super.setSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }
}
