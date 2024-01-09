package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double positionTicks = 0.0;
    public double absoluteTicks = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith() {}
}
