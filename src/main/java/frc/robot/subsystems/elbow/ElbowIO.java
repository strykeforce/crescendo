package frc.robot.subsystems.elbow;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ElbowIO {

  @AutoLog
  public static class ElbowIOInputs {
    public double positionRots = 0.0;
  }

  public default void updateInputs(ElbowIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPct(double percentOutput) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
