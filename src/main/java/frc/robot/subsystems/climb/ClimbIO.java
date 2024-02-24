package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double leftPosRots = 0.0;
    public double rightPosRots = 0.0;
    public double leftVelocity = 0.0;
    public double rightVelocity = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setLeftPos(double position) {}

  public default void setRightPos(double position) {}

  public default void setPct(double percent) {}

  public default void zero() {}

  public default void setSoftLimitsEnabled(boolean enable) {}

  public default void setCurrentLimit(CurrentLimitsConfigs config) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
