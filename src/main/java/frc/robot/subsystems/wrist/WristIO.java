package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double position = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
