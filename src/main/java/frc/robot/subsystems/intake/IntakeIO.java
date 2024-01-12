package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface IntakeIO {

    //@AutoLog
    public static class IntakeIOInputs {
      public double velocityTicksPer100ms = 0.0;
      public boolean isFwdLimitSwitchClosed = false;
      public boolean isRevLimitSwitchClosed = false;
      }

      public default void updateInputs(IntakeIOInputs inputs) {}
      
      public default void setPct(double percentOutput) {}
      
      public default void setSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}
    
      public default void registerWith() {}
}
