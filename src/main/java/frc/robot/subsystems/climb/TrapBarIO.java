package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface TrapBarIO {
  @AutoLog
  public static class TrapBarIOInputs {
    public double leftPos = 0.0;
    public double rightPos = 0.0;
  }

  public default void updateInputs(TrapBarIOInputs inputs) {}

  public default void setPosition(double leftPos, double rightPos) {}
}
