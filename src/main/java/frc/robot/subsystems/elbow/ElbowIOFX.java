package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

  MotionMagicVoltage positionRequst =
      new MotionMagicVoltage(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;
  StatusSignal<Double> absRots;

  public ElbowIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    elbow = new TalonFX(ElbowConstants.kElbowTalonFxId);
    remoteEncoder = new CANcoder(ElbowConstants.kRemoteEncoderID);

    CANcoderConfigurator canCoderConfig = remoteEncoder.getConfigurator();
    canCoderConfig.apply(new CANcoderConfiguration());
    canCoderConfig.apply(ElbowConstants.getCanCoderConfig());

    absSensorInitial = elbow.getPosition().getValue();

    configurator = elbow.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ElbowConstants.getFxConfiguration());

    currPosition = elbow.getPosition();
    currVelocity = elbow.getVelocity();
    absRots = remoteEncoder.getAbsolutePosition();
  }

  //   public int getPulseWidthFor(PWMChannel channel) {
  //     double[] pulseWidthandPeriod = new double[2];
  //     // remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
  //     remoteEncoder.getAbsolutePosition();
  //     return (int)
  //         (ElbowConstants.kFxToMechRatio
  //             / ElbowConstants.kAbsEncoderToMechRatio
  //             * pulseWidthandPeriod[0]
  //             / pulseWidthandPeriod[1]);
  //   }

  @Override
  public void zero() {
    double absoluteRots = absRots.refresh().getValue();

    relSetpointOffset = absoluteRots - RobotConstants.kElbowZero + ElbowConstants.kZeroOffset;

    logger.info("RelSetPoint {}", relSetpointOffset);
    relSetpointOffset =
        relSetpointOffset
            / ElbowConstants.kAbsEncoderToMechRatio
            * ElbowConstants.kFxGearbox
            * ElbowConstants.kFxChain
            * ElbowConstants.kFxPulley;
    logger.info("RelSetPointPostRatio {}", relSetpointOffset);
    elbow.setPosition(relSetpointOffset);

    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}", absRots, RobotConstants.kElbowZero, relSetpointOffset);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    elbow.setControl(positionRequst.withPosition(setpoint));
  }

  @Override
  public void setPct(double percentOutput) {
    elbow.set(percentOutput);
  }

  @Override
  public void updateInputs(ElbowIOInputs inputs) {
    inputs.positionRots = currPosition.refresh().getValue();
    inputs.absRots = absRots.refresh().getValue();
    inputs.velocity = currVelocity.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(elbow, true);
    telemetryService.register(new CancoderMeasureable(remoteEncoder));
  }

  @Override
  public void setCurrentLimit(CurrentLimitsConfigs config) {
    configurator = elbow.getConfigurator();
    configurator.apply(config);
  }
}
