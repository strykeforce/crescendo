package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ShooterConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;

public class ShooterIOFX implements ShooterIO, Checkable {

  public double leftSetpoint = 0.0;

  @HealthCheck
  @Timed(
      percentOutput = {0.2, -0.2, 0.8, 0.0, -0.8},
      duration = 3)
  private TalonFX shooterLeft;

  @HealthCheck
  @Follow(leader = ShooterConstants.kLeftShooterTalonID)
  private TalonFX shooterRight;

  @BeforeHealthCheck
  public boolean followTalons() {
    shooterRight.setControl(new Follower(ShooterConstants.kLeftShooterTalonID, false));
    return true;
  }

  private Logger logger;

  private double setpoint;

  TalonFXConfigurator configurator;
  private MotionMagicVelocityVoltage velocityLeftRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private MotionMagicVelocityVoltage velocityRightRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  StatusSignal<Double> curLeftVelocity;
  StatusSignal<Double> curRightVelocity;

  public ShooterIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    shooterLeft = new TalonFX(ShooterConstants.kLeftShooterTalonID);
    shooterRight = new TalonFX(ShooterConstants.kRightShooterTalonID);

    configurator = shooterLeft.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ShooterConstants.getLeftShooterConfig());
    // shooterLeft.getClosedLoopError().setUpdateFrequency(200.0);
    // shooterLeft.getClosedLoopReference().setUpdateFrequency(200.0);

    configurator = shooterRight.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // factory default
    configurator.apply(ShooterConstants.getRightShooterConfig());

    curLeftVelocity = shooterLeft.getVelocity();
    curRightVelocity = shooterRight.getVelocity();
  }

  @Override
  public String getName() {
    return "Shooter";
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftSetpoint = leftSetpoint;
    inputs.velocityLeft = curLeftVelocity.refresh().getValue();
    inputs.velocityRight = curRightVelocity.refresh().getValue();
  }

  @Override
  public void setPct(double percentOutput) {
    shooterLeft.setControl(dutyCycleRequest.withOutput(percentOutput));
    shooterRight.setControl(
        dutyCycleRequest.withOutput(
            percentOutput)); // FIXME: figure out which needs to be inverted to shoot = positive
  }

  @Override
  public void setSpeed(double speed) {
    leftSetpoint = speed;
    shooterLeft.setControl(velocityLeftRequest.withVelocity(speed));
    shooterRight.setControl(
        velocityRightRequest.withVelocity(
            speed)); // FIXME: figure out which needs to be inverted to shoot = positive
  }

  @Override
  public void setLeftSpeed(double speed) {
    leftSetpoint = speed;
    shooterLeft.setControl(velocityLeftRequest.withVelocity(speed));
  }

  @Override
  public void setRightSpeed(double speed) {
    shooterRight.setControl(velocityRightRequest.withVelocity(speed));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shooterLeft, true);
    telemetryService.register(shooterRight, true);
  }
}
