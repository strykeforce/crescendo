package frc.robot.subsystems.led;

import java.util.Random;
import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LedConstants;

public class LedSubsystem extends MeasurableSubsystem{

    private LedState currState = LedState.OFF;
    private AddressableLED led = new AddressableLED(LedConstants.kLedPort);
    private AddressableLEDBuffer ledBuffer;

    public LedSubsystem() {
        ledBuffer = new AddressableLEDBuffer(LedConstants.kLedLength);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public LedState getState() {
        return currState;
    }

    private void setState(LedState state) {
        currState = state;
    }

    public void setColor(int r, int g, int b) {
        setState(LedState.SOLID);
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setColor(Color color) {
        setState(LedState.SOLID);
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setGreen() {
        setColor(LedConstants.kGreen);
    }

    public void setBlue() {
        setColor(LedConstants.kBlue);
    }

    public void setFlaming() {
        setState(LedState.FLAMING);
    }
    
    public void setOff() {
        setColor(new Color());
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
            case OFF:
                break;
            default:
                setOff();
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
