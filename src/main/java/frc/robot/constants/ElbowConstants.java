package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public final class ElbowConstants {
  public static final int kElbowTalonFxId = 30;
  public static final int kRemoteEncoderID = 31;
  public static final double kCloseEnoughTicks = 100;
  public static final double kMaxPivotTicks = 0;
  public static final double kMinPivotTicks = 1000;
  // public static final double kElbowZeroTicks = 0;

  public static final double kAbsEncoderToMechRatio = 50;
  public static final double kFxToMechRatio = 2;

  public static TalonFXConfiguration getFxConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    

    return config;
  }
}
