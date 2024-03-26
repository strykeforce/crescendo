package frc.robot.subsystems.magazine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import frc.robot.constants.MagazineConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;

public class MagazineIOFX implements MagazineIO, Checkable {
  @HealthCheck
  @Timed(
      percentOutput = {0.2, -0.2, 0.9, -0.9},
      duration = 3)
  private TalonFX magazine;

  private Logger logger;

  private double setpoint = 0.0;

  private TalonFXConfigurator configurator;
  private MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  private StatusSignal<Double> currVelocity;
  private StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  private StatusSignal<ReverseLimitValue> revLimitSwitch;

  public MagazineIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    magazine = new TalonFX(MagazineConstants.kMagazineFalconID);

    configurator = magazine.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(MagazineConstants.getMagazineConfig());

    currVelocity = magazine.getVelocity();
    fwdLimitSwitch = magazine.getForwardLimit();
    revLimitSwitch = magazine.getReverseLimit();
  }

  @Override
  public String getName() {
    return "Magazine";
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 0;
    inputs.setpoint = setpoint;
  }

  @Override
  public void enableFwdLimitSwitch(boolean enabled) {
    magazine
        .getConfigurator()
        .apply(
            MagazineConstants.getMagazineConfig()
                .HardwareLimitSwitch
                .withForwardLimitEnable(enabled));
  }

  @Override
  public void enableRevLimitSwitch(boolean enabled) {
    magazine
        .getConfigurator()
        .apply(
            MagazineConstants.getMagazineConfig()
                .HardwareLimitSwitch
                .withReverseLimitEnable(enabled));
  }

  @Override
  public void setPct(double percentOutput) {
    magazine.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    if (speed == 0.0) magazine.setControl(dutyCycleRequest.withOutput(speed));
    else magazine.setControl(velocityRequest.withVelocity(speed));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(magazine, true);
  }
}
