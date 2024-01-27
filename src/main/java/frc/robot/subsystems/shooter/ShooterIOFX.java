package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ShooterConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ShooterIOFX implements ShooterIO {
  private TalonFX shooterLeft;
  private TalonFX shooterRight;
  private Logger logger;

  private double setpoint;

  TalonFXConfigurator configurator;
  private VelocityDutyCycle velocityLeftRequest =
      new VelocityDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  private VelocityDutyCycle velocityRightRequest =
      new VelocityDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  StatusSignal<Double> curLeftVelocity;
  StatusSignal<Double> curRightVelocity;

  public ShooterIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    shooterLeft = new TalonFX(ShooterConstants.kLeftShooterTalonID);
    shooterRight = new TalonFX(ShooterConstants.kRightShooterTalonID);

    configurator = shooterLeft.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ShooterConstants.getShooterConfig());

    configurator = shooterRight.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ShooterConstants.getShooterConfig());

    curLeftVelocity = shooterLeft.getVelocity();
    curRightVelocity = shooterRight.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityLeft = curLeftVelocity.refresh().getValue();
    inputs.velocityRight = curRightVelocity.refresh().getValue();
  }

  @Override
  public void setPct(double percentOutput) {
    shooterLeft.setControl(dutyCycleRequest.withOutput(-percentOutput));
    shooterRight.setControl(
        dutyCycleRequest.withOutput(
            percentOutput)); // FIXME: figure out which needs to be inverted to shoot = positive
  }

  @Override
  public void setSpeed(double speed) {
    shooterLeft.setControl(dutyCycleRequest.withOutput(-speed));
    shooterRight.setControl(
        dutyCycleRequest.withOutput(
            speed)); // FIXME: figure out which needs to be inverted to shoot = positive
  }

  @Override
  public void setLeftSpeed(double speed) {
    shooterLeft.setControl(dutyCycleRequest.withOutput(speed));
  }

  @Override
  public void setRightSpeed(double speed) {
    shooterRight.setControl(dutyCycleRequest.withOutput(speed));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shooterLeft, true);
  }
}
