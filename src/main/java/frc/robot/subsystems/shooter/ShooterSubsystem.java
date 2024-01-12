package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.standards.*;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {

  // Private Variables
  TalonFX shooterMotor;
  double setpoint = 0.0;
  ShooterStates curState = ShooterStates.IDLE;

  // Constructor
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.kShooterTalonID);
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(shooterMotor.getSelectedSensorVelocity() - setpoint)
        < ShooterConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    shooterMotor.set(TalonFXControlMode.Velocity, speed);
  }

  public ShooterStates getState() {
    return curState;
  }

  public void setState(ShooterStates state) {
    curState = state;
  }

  // Helper Methods

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case SHOOT:
        break;

      case SPEEDUP:
        if (atSpeed()) {
          curState = ShooterStates.SHOOT;
        }
        break;

      case IDLE:
        break;
    }
  }
  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }

  // State Enum
  public enum ShooterStates {
    SHOOT,
    SPEEDUP,
    IDLE
  }
}
