package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityLeft;
    public double velocityRight;
    public double leftSetpoint = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSpeed(double speed) {}

  public default void setLeftSpeed(double speed) {}

  public default void setRightSpeed(double speed) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void enableFwdLimitSwitch(boolean enabled) {}
}
