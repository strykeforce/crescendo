package frc.robot.subsystems.magazine;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface MagazineIO {
  public static class MagazineIOInputs {
    public double velocity;
    public double position;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith() {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 2767.0;
  }
}
