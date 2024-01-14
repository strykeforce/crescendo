package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ElbowEncoderIO {

  @AutoLog
  public static class ElbowEncoderIOInputs {
    public double absolutePercentage = 0.0;
  }

  public default void updateInputs(ElbowEncoderIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
