package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import frc.robot.constants.ElbowConstants;
import frc.robot.constants.RobotConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.AfterHealthCheck;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CancoderMeasureable;

public class ElbowIOFX implements ElbowIO, Checkable {
  private Logger logger;

  @HealthCheck
  @Position(
      percentOutput = {0.1, -0.1},
      encoderChange = 25)
  private TalonFX elbow;

  private CANcoder remoteEncoder;

  private double absSensorInitial;
  private double relSetpointOffset;
  private double setpointOffset;
  private double setpoint;

  TalonFXConfigurator configurator;

  MotionMagicVoltage positionRequst =
      new MotionMagicVoltage(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Double> currPosition;
  StatusSignal<Double> currVelocity;
  StatusSignal<Double> absRots;
  StatusSignal<ReverseLimitValue> revLim;

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
    revLim = elbow.getReverseLimit();
  }

  @Override
  public String getName() {
    return "Elbow";
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
    absSensorInitial = elbow.getPosition().getValue();

    relSetpointOffset = absSensorInitial - RobotConstants.kElbowZero;
    setpointOffset =
        RobotConstants.kElbowZeroPos - RobotConstants.kElbowZeroPos - relSetpointOffset;
    logger.info("REAL ZERO");
    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}, setpointOffset: {}",
        absSensorInitial,
        RobotConstants.kElbowZero,
        relSetpointOffset,
        setpointOffset);
  }

  @Override
  public void zeroBlind() {
    absSensorInitial = elbow.getPosition().getValue();

    // relSetpointOffset = 0.0;
    // setpointOffset = absSensorInitial - RobotConstants.kElbowZeroPos - relSetpointOffset;
    relSetpointOffset = 0;
    setpointOffset = 0;
    elbow.setPosition(RobotConstants.kElbowZeroPos);
    logger.info("BLIND ZERO");
    logger.info("Abs: {}, Zero Pos: {}", absSensorInitial, RobotConstants.kElbowZeroPos);
  }

  @Override
  public void zeroRecovery() {
    double absoluteRots = absRots.refresh().getValue();
    double curPos = elbow.getPosition().getValue();

    relSetpointOffset = absoluteRots - RobotConstants.kElbowRecoveryZero;
    logger.info("RECOVERY ZERO");

    logger.info("RelSetPoint {}", relSetpointOffset);
    relSetpointOffset =
        relSetpointOffset
            / ElbowConstants.kAbsEncoderToMechRatio
            * ElbowConstants.kFxGearbox
            * ElbowConstants.kFxChain
            * ElbowConstants.kFxPulley;
    logger.info("RelSetPointPostRatio {}", relSetpointOffset);
    // elbow.setPosition(relSetpointOffset);
    setpointOffset = curPos - relSetpointOffset;

    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}, setpointOffset: {}",
        absRots,
        RobotConstants.kElbowRecoveryZero,
        relSetpointOffset,
        setpointOffset);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position + setpointOffset;

    elbow.setControl(positionRequst.withPosition(setpoint));
  }

  @Override
  public void setPct(double percentOutput) {
    elbow.set(percentOutput);
  }

  @Override
  public void updateInputs(ElbowIOInputs inputs) {
    inputs.encoderPosRots = currPosition.refresh().getValue();
    inputs.positionRots = currPosition.refresh().getValue() - setpointOffset;
    inputs.absRots = absRots.refresh().getValue();
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.revLimitClosed = revLim.refresh().getValue() == ReverseLimitValue.ClosedToGround;
    inputs.setpoint = setpoint;
  }

  @Override
  public void setCurrentLimit(CurrentLimitsConfigs config) {
    configurator = elbow.getConfigurator();
    configurator.apply(config);
  }

  @Override
  public void configMotionMagic(MotionMagicConfigs config) {
    configurator = elbow.getConfigurator();
    configurator.apply(config);
  }

  @Override
  public void configHardwareLimit(HardwareLimitSwitchConfigs config) {
    configurator = elbow.getConfigurator();
    configurator.apply(config);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(elbow, true);
    telemetryService.register(new CancoderMeasureable(remoteEncoder));
  }

  @BeforeHealthCheck
  @AfterHealthCheck
  public boolean goToZero() {
    setPosition(0.0);
    return Math.abs(currPosition.refresh().getValue() - setpointOffset)
        <= ElbowConstants.kCloseEnoughRots;
  }
}
