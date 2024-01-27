package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class MagazineConstants {
  public static final int kMagazineFalconID = 25;
  public static final double kCloseEnough = 300;
  public static final double kShootCloseEnough = 500;
  // public static final double kFeedingSpeed = 0.5;
  public static final double kShootSpeed = 0;
  public static final int kMinBeamBreaks = 3;
  public static final double kIntakingSpeed = -0.5;
  public static final double kEmptyingSpeed = -0.75;

  public static final double kPodiumPrepareSpeed = 0.1;

  public static final TalonFXConfiguration getMagazineConfig() {
    return new TalonFXConfiguration();
  }

  public static final SupplyCurrentLimitConfiguration getMagazineSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration();
  }
}
