package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.standards.OpenLoopSubsystem;

public class IntakeSubsystem extends SubsystemBase implements OpenLoopSubsystem {
    private final IntakeIO io;
    private IntakeState curIntakeState = IntakeState.NONE;
    private IntakeState desiredState;
    private IntakeState finalState;
    private IntakeConstants intakeConstants;

    private boolean beamBroken = false; 
    private double beamBreakStableCounts = 0; 


    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public IntakeState getState() {
        return curIntakeState;
    }

    public boolean getObjectStatus() {
        if(curIntakeState == IntakeState.HAS_PIECE) {
            return true;
        } else {
            return false;
        }
    } 

    public void toIntaking() {
        //logger.info("To Intaking");
        curIntakeState = IntakeState.INTAKING;
        setPercent();
    }

    public void intakeOpenLoop(double percentOutput) {
        // intakeFalcon.set(ControlMode.PercentOutput, percentOutput)
    }

    @Override
    public void setPercent() {
        // intakeFalcon.set(ControlMode.PercentOutput, kIntakePercentOutput);
    }

    public boolean isBeamBroken() {
        return false;
    }

    @Override
    public void periodic() {

        switch(curIntakeState) {
            case HAS_PIECE:
            // has a gamepiece, disables intake
                break;
            case INTAKING:
                if (isBeamBroken()) {
                    intakeOpenLoop(0);
                    curIntakeState = IntakeState.HAS_PIECE;
                }
                break;
        }
    }

    public enum IntakeState {
        HAS_PIECE,
        INTAKING,
        NONE
    }
}
