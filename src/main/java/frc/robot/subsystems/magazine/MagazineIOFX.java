package frc.robot.subsystems.magazine;

import javax.annotation.OverridingMethodsMustInvokeSuper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MagazineIOFX implements MagazineIO {
    private TalonFX magazine;

    public MagazineIOFX () {
        magazine = new TalonFX(MagazineConstants.kMagazineFalconID);
        magazine.configFactoryDefault();
        magazine.configAllSettings(MagazineConstants.getMagazineConfig());
        magazine.configSupplyCurrentLimit(MagazineConstants.getMagazineSupplyLimitConfig());
        magazine.setNeutralMode(NeutralMode.Brake);
    }
    
    @Override
    public void updateInputs(MagazineIOInputs inputs) {
        inputs.position = magazine.getSelectedSensorPosition();
        inputs.velocity = magazine.getSelectedSensorVelocity();
        inputs.isFwdLimitSwitchClosed = magazine.isFwdLimitSwitchClosed() == 1;
        inputs.isRevLimitSwitchClosed = magazine.isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void setPct(double percentOutput) {
        magazine.set(TalonFXControlMode.PercentOutput, percentOutput);
    }
    
    @Override
    public void setSpeed(double speed) {
        magazine.set(TalonFXControlMode.Velocity, speed);
    }

    @Override
    public double getSpeed() {
        return magazine.getSelectedSensorVelocity();
    }

    @Override
    public void registerWith() {
        // TODO Auto-generated method stub
        MagazineIO.super.registerWith();
    }

    @Override
    public void setSupplyCurrentLimit(SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
        // TODO Auto-generated method stub
        MagazineIO.super.setSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    }
}
