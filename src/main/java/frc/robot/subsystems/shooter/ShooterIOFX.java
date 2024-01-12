package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ShooterIOFX implements ShooterIO {
  private TalonFX shooter;

  public ShooterIOFX() {
    shooter = new TalonFX(ShooterConstants.kShooterTalonID);
    shooter.configFactoryDefault();
    shooter.configAllSettings(ShooterConstants.getShooterConfig());
    shooter.configSupplyCurrentLimit(ShooterConstants.getShooterSupplyLimitConfig());
    shooter.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocity = shooter.getSelectedSensorVelocity();
    inputs.position = shooter.getSelectedSensorPosition();
    inputs.isFwdLimitSwitchClosed = shooter.isFwdLimitSwitchClosed() == 1;
    inputs.isRevLimitSwitchClosed = shooter.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void setPct(double percentOutput) {
    shooter.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSpeed(double speed) {
    shooter.set(TalonFXControlMode.Velocity, speed);
  }

  @Override
  public double getSpeed() {
    return shooter.getSelectedSensorVelocity();
  }

  @Override
  public void registerWith() {
    // TODO Auto-generated method stub
    ShooterIO.super.registerWith();
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    // TODO Auto-generated method stub
    ShooterIO.super.setSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }
}
