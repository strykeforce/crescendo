package frc.robot.subsystems.intake;

public interface IntakeIO {
    public static class IntakeIOInputs {
        public double velocity = 0.0;
      }
    
      public default void updateInputs(IntakeIOInputs inputs) {}
    
      public default void registerWith() {}
}
