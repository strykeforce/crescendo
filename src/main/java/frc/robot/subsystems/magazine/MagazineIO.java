package frc.robot.subsystems.magazine;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface MagazineIO {

  @AutoLog
  public static class MagazineIOInputs {
    public double velocity = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
    public boolean isSecondFwdLimitSwitchClosed = false;
  }

  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSpeed(double speed) {}

  // public default void setFwdLimitSwitchEnabled(boolean enabled) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
