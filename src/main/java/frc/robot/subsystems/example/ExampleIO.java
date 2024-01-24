package frc.robot.subsystems.example;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ExampleIO {

  @AutoLog
  public static class ExampleIOInputs {
    public double position = 0.0;
    public double absPos = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
