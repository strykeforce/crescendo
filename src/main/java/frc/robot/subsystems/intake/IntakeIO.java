package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
