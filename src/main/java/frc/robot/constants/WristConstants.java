package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class WristConstants {
  public static final int kWristTalonFxId = 0;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  public static final double kWristZeroTicks = 0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    return new TalonSRXConfiguration();
  }
}
