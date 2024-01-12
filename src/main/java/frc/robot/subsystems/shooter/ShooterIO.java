package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface ShooterIO {
  public static class ShooterIOInputs {
    public double velocity;
    public double position;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith() {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 2767.0;
  }
}
