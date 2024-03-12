package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ForkIO {

  @AutoLog
  public static class ForkIOInputs {
    public double leftPosTicks = 0.0;
    public double rightPosTicks = 0.0;
    public double leftVelTicks = 0.0;
    public double rightVelTicks = 0.0;
  }

  public default void updateInputs(ForkIOInputs inputs) {}

  public default void zeroLeft() {}

  public default void zeroRight() {}

  public default void setPct(double percent) {}

  public default void setLeftPct(double percent) {}

  public default void setRightPct(double percent) {}

  public default void setPosition(double position) {}

  public default void setLeftPos(double position) {}

  public default void setRightPos(double position) {}

  public default void enableSoftLimits(boolean enable) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
