package frc.robot.subsystems.example;

public interface ExampleIO {

  public static class ExampleIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith() {}
}
