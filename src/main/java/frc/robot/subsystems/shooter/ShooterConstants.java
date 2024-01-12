package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ShooterConstants {
  public static final int kShooterTalonID = -1;
  public static final double kCloseEnough = 100;

  public static final TalonFXConfiguration getShooterConfig() {
    return new TalonFXConfiguration();
  }

  public static final SupplyCurrentLimitConfiguration getShooterSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration();
  }
}
