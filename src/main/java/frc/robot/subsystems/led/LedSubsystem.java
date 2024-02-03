package frc.robot.subsystems.led;

import java.util.Random;
import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.constants.LedConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class LedSubsystem extends MeasurableSubsystem{

    private IntakeSubsystem intakeSubsystem;
    private LedState currState = LedState.OFF;
    private AddressableLED led = new AddressableLED(LedConstants.kLedPort);
    private AddressableLEDBuffer ledBuffer;

    public LedSubsystem(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        ledBuffer = new AddressableLEDBuffer(LedConstants.kLedLength);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public LedState getState() {
        return currState;
    }

    @Override
    public void periodic() {
        if (intakeSubsystem.getState() == IntakeState.INTAKING) {
            currState = LedState.FLAMING;
        } else if (intakeSubsystem.getState() == IntakeState.HAS_PIECE) {
            currState = LedState.RECIEVED;
        } else {
            currState = LedState.OFF;
        }

        switch (currState) {
            case FLAMING:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 250, (int) (Math.random() * 185), 0);
                }
                break;
            case RECIEVED:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 5, 250, 21);
                }
                break;
            default:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
                break;
        }
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(
            new Measure("State", () -> getState().ordinal())
        );
    }

    public enum LedState {
        OFF,
        FLAMING,
        RECIEVED
    }
    
}
