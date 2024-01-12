package frc.robot.subsystems.magazine;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;

public class MagazineSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem{
    // Private Variables
    TalonSRX magazineMotor;
    double setpoint = 0.0;
    MagazineStates curState = MagazineStates.EMPTY;
    

  // Constructor
  public MagazineSubsystem() {
    magazineMotor = new TalonSRX(MagazineConstants.kMagazineMotorID);
  }

  // Getter/Setter Methods

  @Override
  public double getSpeed() {
    return magazineMotor.getSelectedSensorVelocity();
  }

  @Override
  public boolean atSpeed() {
    return Math.abs(magazineMotor.getSelectedSensorVelocity() - setpoint)
        < MagazineConstants.kCloseEnough;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    magazineMotor.set(TalonSRXControlMode.Velocity, speed);
  }

  public MagazineStates getState() {
    return curState;
  }

  public void setState(MagazineStates state) {
    curState = state;
  }

  // Helper Methods

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case EMPTY:
    
      break;
      case FULL:
      
      break;
      case INTAKING:
      
      break;
      case EMPTYING:
      
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
  public enum MagazineStates {
    EMPTY, FULL, INTAKING, EMPTYING
  }
}
