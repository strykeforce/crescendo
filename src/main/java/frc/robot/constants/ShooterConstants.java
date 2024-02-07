package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class ShooterConstants {
  public static final int kLeftShooterTalonID = 40;
  public static final int kRightShooterTalonID = 41;
  public static final double kCloseEnough = 100;
  public static final double kShootTime = 1.0;
  public static final double kPodiumSpeed = 0;

  public static final TalonFXConfiguration getShooterConfig() {
    
    return new TalonFXConfiguration();
  }

  public static final SupplyCurrentLimitConfiguration getShooterSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration();
  }
}
