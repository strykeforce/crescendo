package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ExampleConstants;
import frc.robot.constants.IntakeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class IntakeIOFX implements IntakeIO {

  private Logger logger;
  private TalonFX intake;

  private final double absSensorInitial;
  private double relSetpointOffset;
  private double setpoint;

  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;

  public IntakeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    intake = new TalonFX(IntakeConstants.kIntakeFxId);
    absSensorInitial = intake.getPosition().getValue();

    configurator = intake.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(ExampleConstants.getFXConfig());

    currPosition = intake.getPosition();
    currVelocity = intake.getVelocity();
  }

  @Override
  public void setPct(double position) {
    setpoint = position - relSetpointOffset;
    intake.setControl(positionRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = currPosition.refresh().getValue();
    inputs.velocity = currVelocity.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(intake, true);
  }
}
