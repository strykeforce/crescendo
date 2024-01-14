package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ElbowConstants;
import frc.robot.constants.WristConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class WristIOFX implements WristIO {
  private Logger logger;
  private TalonFX wrist;

  private double absSesorInitial;
  private double relSetpointOffset;
  private double setpoint;

  TalonFXConfigurator configurator;
  MotionMagicDutyCycle positionRequst =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;

  public WristIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    wrist = new TalonFX(WristConstants.kElbowTalonFxId);
    absSesorInitial = wrist.getPosition().getValue();

    configurator = wrist.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(WristConstants.getFxConfiguration());

    currPosition = wrist.getPosition();
    currVelocity = wrist.getVelocity();
  }

  @Override
  public void zero() {
    relSetpointOffset = absSesorInitial - WristConstants.kElbowZeroTicks;
    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}",
        absSesorInitial,
        ElbowConstants.kElbowZeroTicks,
        relSetpointOffset);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position - relSetpointOffset;
    wrist.setControl(positionRequst.withPosition(position));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.positionTicks = currPosition.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(wrist, true);
  }
}
