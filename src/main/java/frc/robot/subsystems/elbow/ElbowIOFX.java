package frc.robot.subsystems.elbow;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ElbowConstants;
import frc.robot.constants.RobotConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CancoderMeasureable;

public class ElbowIOFX implements ElbowIO {
  private Logger logger;
  private TalonFX elbow;
  private CANcoder remoteEncoder;

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
    remoteEncoder = new CANcoder(ElbowConstants.kRemoteEncoderID);

    absSensorInitial = elbow.getPosition().getValue();

    configurator = elbow.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ElbowConstants.getFxConfiguration());

    currPosition = elbow.getPosition();
    currVelocity = elbow.getVelocity();
  }

  public int getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    // remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    remoteEncoder.getPosition();
    return (int)
        (ElbowConstants.kFxToMechRatio
            / ElbowConstants.kAbsEncoderToMechRatio
            * pulseWidthandPeriod[0]
            / pulseWidthandPeriod[1]);
  }

  @Override
  public void zero() {
    int absoluteTicks = getPulseWidthFor(PWMChannel.PWMChannel0);

    relSetpointOffset = absoluteTicks - ElbowConstants.kElbowZeroTicks;
    elbow.setPosition(relSetpointOffset);

    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}",
        absSensorInitial,
        RobotConstants.kElbowZero,
        relSetpointOffset);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position + relSetpointOffset;
    elbow.setControl(positionRequst.withPosition(setpoint));
  }

  @Override
  public void setPct(double percentOutput) {
    elbow.set(percentOutput);
  }

  @Override
  public void updateInputs(ElbowIOInputs inputs) {
    inputs.positionRots = currPosition.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(elbow, true);
    telemetryService.register(new CancoderMeasureable(remoteEncoder));
  }
}
