package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public double odometryX = 0.0;
    public double odometryY = 0.0;
    public double gyroRotation = 0.0;
    public double odometryRotation2D = 0.0;
  }

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
