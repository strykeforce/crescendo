package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ElbowConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ElbowIOFX implements ElbowIO {
  private Logger logger;
  private TalonFX elbow;

  private double absSensorInitial;
  private double relSetpointOffset;
  private double setpoint;

  TalonFXConfigurator configurator;
  MotionMagicDutyCycle positionRequst =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;

  public ElbowIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    elbow = new TalonFX(ElbowConstants.kElbowTalonFxId);
    absSensorInitial = elbow.getPosition().getValue();

    configurator = elbow.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ElbowConstants.getFxConfiguration());

    currPosition = elbow.getPosition();
    currVelocity = elbow.getVelocity();
  }

  @Override
  public void zero() {
    relSetpointOffset = absSensorInitial - ElbowConstants.kElbowZeroTicks;

    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}",
        absSensorInitial,
        ElbowConstants.kElbowZeroTicks,
        relSetpointOffset);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position - relSetpointOffset;
    elbow.setControl(positionRequst.withPosition(setpoint));
  }

  @Override
  public void updateInputs(ElbowIOInputs inputs) {
    inputs.positionRots = currPosition.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(elbow, true);
  }
}
