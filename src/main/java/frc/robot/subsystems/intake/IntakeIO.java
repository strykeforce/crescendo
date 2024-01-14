package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith() {}

  public default void registerWith(TelemetryService telemetryService) {}
  ;
}
