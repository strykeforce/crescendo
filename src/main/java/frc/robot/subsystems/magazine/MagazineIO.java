package frc.robot.subsystems.magazine;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface MagazineIO {

  @AutoLog
  public static class MagazineIOInputs {
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 2767.0;
  }
}
