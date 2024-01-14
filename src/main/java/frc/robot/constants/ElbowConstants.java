package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElbowConstants {
  public static final int kElbowTalonFxId = 0;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  public static final double kElbowZeroTicks = 0;

  public static TalonFXConfiguration getFxConfiguration() {
    return new TalonFXConfiguration();
  }
}
