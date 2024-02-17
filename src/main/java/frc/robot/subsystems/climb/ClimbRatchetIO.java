package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbRatchetIO {

  @AutoLog
  public static class ClimbRatchetIOInputs {
    public double leftPos = 0.0;
    public double rightPos = 0.0;
  }

  public default void updateInputs(ClimbRatchetIOInputs inputs) {}

  public default void setPosition(double leftPos, double rightPos) {}
}
