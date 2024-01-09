package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface PivotEncoderIO {

  @AutoLog
  public static class PivotEncoderIOInputs {
    public double absolutePercentage = 0.0;
  }

  public default void updateInputs(PivotEncoderIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
