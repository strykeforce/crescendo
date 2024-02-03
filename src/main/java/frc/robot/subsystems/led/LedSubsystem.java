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

    public void setColor(int r, int g, int b) {
        currState = LedState.SOLID;
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setGreen() {
        setColor(0, 255, 0);
    }

    public void setFlaming() {
        currState = LedState.FLAMING;
    }
    
    public void setOff() {
        currState = LedState.OFF;
    }

    @Override
    public void periodic() {

        switch (currState) {
            case FLAMING:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 250, (int) (Math.random() * 185), 0);
                }
                break;
            case SOLID:
                break;
            default:
                setColor(0, 0, 0);
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
        SOLID,
        FLAMING
    }
    
}
