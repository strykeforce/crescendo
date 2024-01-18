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
  private TalonFX shooterLeft;
  private TalonFX shooterRight;
  private Logger logger;

  private double setpoint;
  private final double absSensorIntial;

  TalonFXConfigurator configurator;
  private VelocityDutyCycle velocityLeftRequest =
      new VelocityDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  private VelocityDutyCycle velocityRightRequest =
      new VelocityDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> curLeftPosition;
  StatusSignal<Double> curLeftVelocity;
  StatusSignal<Double> curRightPosition;
  StatusSignal<Double> curRightVelocity;
  StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  StatusSignal<ReverseLimitValue> revLimitSwitch;

  public ShooterIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    shooterLeft = new TalonFX(ShooterConstants.kLeftShooterTalonID);
    shooterRight = new TalonFX(ShooterConstants.kRightShooterTalonID);
    absSensorIntial = shooterLeft.getPosition().getValue();

    configurator = shooterLeft.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(ShooterConstants.getShooterConfig());

    configurator = shooterRight.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(ShooterConstants.getShooterConfig());

    curLeftPosition = shooterLeft.getPosition();
    curLeftVelocity = shooterLeft.getVelocity();
    fwdLimitSwitch = shooterLeft.getForwardLimit();
    revLimitSwitch = shooterLeft.getReverseLimit();

    curRightPosition = shooterRight.getPosition();
    curRightVelocity = shooterRight.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityLeft = curLeftVelocity.refresh().getValue();
    inputs.positionLeft = curLeftPosition.refresh().getValue();
    inputs.velcoityRight = curRightVelocity.refresh().getValue();
    inputs.positionRight = curRightPosition.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 1;
  }

  @Override
  public void setPct(double percentOutput) {
    shooterLeft.set(percentOutput);
    shooterRight.set(percentOutput);
  }

  @Override
  public void setSpeed(double speed) {
    shooterLeft.setControl(velocityLeftRequest.withVelocity(speed));
    shooterRight.setControl(velocityRightRequest.withVelocity(speed));
  }

  @Override
  public void setLeftSpeed(double speed) {
    shooterLeft.setControl(velocityLeftRequest.withVelocity(speed));
  }

  @Override
  public void setRightSpeed(double speed) {
    shooterRight.setControl(velocityRightRequest.withVelocity(speed));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shooterLeft, true);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    // TODO Auto-generated method stub
    ShooterIO.super.setSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }
}
