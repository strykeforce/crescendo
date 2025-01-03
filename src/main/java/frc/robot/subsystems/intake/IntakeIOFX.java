package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.IntakeConstants;
import java.util.function.BooleanSupplier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;

public class IntakeIOFX implements IntakeIO, Checkable {

  private Logger logger;

  @HealthCheck
  @Timed(
      percentOutput = {0.5, -0.5},
      duration = 3)
  private TalonFX intake;

  TalonFXConfigurator configurator;
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  StatusSignal<Double> currVelocity;
  BooleanSupplier fwdLimitSupplier = () -> false;

  public IntakeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    intake = new TalonFX(IntakeConstants.kIntakeFxId);

    configurator = intake.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(IntakeConstants.getFXConfig());

    currVelocity = intake.getVelocity();
  }

  @Override
  public String getName() {
    return "Intake";
  }

  @Override
  public void setPct(double percentOutput) {
    intake.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void setFwdLimitSwitchSupplier(BooleanSupplier fwdLimitSupplier) {
    this.fwdLimitSupplier = fwdLimitSupplier;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSupplier.getAsBoolean();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(intake, true);
  }
}
