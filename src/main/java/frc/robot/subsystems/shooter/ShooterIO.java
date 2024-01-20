package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityLeft;
    public double positionLeft;
    public double velcoityRight;
    public double positionRight;
    public boolean isFwdLimitSwitchClosed = false;
    public boolean isRevLimitSwitchClosed = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setSpeed(double speed) {}

  public default void setLeftSpeed(double speed) {}

  public default void setRightSpeed(double speed) {}
}
