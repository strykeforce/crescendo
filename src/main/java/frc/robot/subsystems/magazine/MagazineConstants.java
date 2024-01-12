package frc.robot.subsystems.magazine;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class MagazineConstants {
  public static final int kMagazineFalconID = 0;
  public static final double kCloseEnough = 300;

  public static final TalonFXConfiguration getMagazineConfig() {
    return new TalonFXConfiguration();
  }

  public static final SupplyCurrentLimitConfiguration getMagazineSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration();
  }
}
