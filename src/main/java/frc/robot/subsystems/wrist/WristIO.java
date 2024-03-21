package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double position = 0.0;
    public boolean isRevLimitSwitch = false;
    public boolean isFwdLimitSwitchClosed = false;
    public double setpoint = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPct(double percentOutput) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void forceWristPos(double pos) {}
}
