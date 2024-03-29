package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ElbowIO {

  @AutoLog
  public static class ElbowIOInputs {
    public double positionRots = 0.0;
    public double encoderPosRots = 0.0;
    public double absRots = 0.0;
    public double velocity = 0.0;
    public boolean revLimitClosed = false;
  }

  public default void updateInputs(ElbowIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setPct(double percentOutput) {}

  public default void zero() {}

  public default void zeroBlind() {}

  public default void zeroRecovery() {}

  public default void configMotionMagic(MotionMagicConfigs config) {}

  public default void configHardwareLimit(HardwareLimitSwitchConfigs config) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setCurrentLimit(CurrentLimitsConfigs configs) {}
}
