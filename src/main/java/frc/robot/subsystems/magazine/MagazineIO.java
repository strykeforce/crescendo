package frc.robot.subsystems.magazine;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface MagazineIO {
    public static class MagazineIOInputs {
        public double velocity;
        public double position; 
        public double setpoint;
    }
    public default void updateInputs(MagazineIOInputs inputs) {}
      
    public default void setPct(double percentOutput) {}
      
    public default void setSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}
    
    public default void registerWith() {}
}
