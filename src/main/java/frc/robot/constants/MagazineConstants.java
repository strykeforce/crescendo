package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MagazineConstants {
  public static final int kMagazineFalconID = 0;
  public static final double kCloseEnough = 300;
  public static double kIntakingSpeed;
  public static double kEmptyingSpeed;

  public static final TalonFXConfiguration getMagazineConfig() {
    return new TalonFXConfiguration();
  }

  public static final SupplyCurrentLimitConfiguration getMagazineSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration();
  }
}
