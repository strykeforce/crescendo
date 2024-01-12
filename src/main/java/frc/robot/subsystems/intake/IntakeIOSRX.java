package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.IntakeConstants;

public class IntakeIOSRX implements IntakeIO{
    private TalonSRX intake;

    public IntakeIOSRX() {
        intake = new TalonSRX(IntakeConstants.kIntakeTalonID);
        intake.configFactoryDefault();
        intake.configAllSettings(IntakeConstants.getIntakeConfig());
        intake.configSupplyCurrentLimit(IntakeConstants.getExampleSupplyLimitConfig());
        intake.setNeutralMode(NeutralMode.Brake);
  }
    @Override
    public void setPct(double percentOutput) {
        intake.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        inputs.velocityTicksPer100ms = intake.getSelectedSensorVelocity();
        inputs.isFwdLimitSwitchClosed = intake.isFwdLimitSwitchClosed() == 1.0;
        inputs.isRevLimitSwitchClosed = intake.isRevLimitSwitchClosed() == 1.0;
    }

    @Override
    public void setSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

    @Override
    public void registerWith() {}
}   
