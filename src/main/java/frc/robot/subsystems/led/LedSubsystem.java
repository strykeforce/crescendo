package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LedConstants;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LedSubsystem extends MeasurableSubsystem {

  private LedState currState = LedState.OFF;

  private AddressableLED ledR = new AddressableLED(LedConstants.kRightLedPort);
  private AddressableLEDBuffer ledBufferR;

  private AddressableLED ledL = new AddressableLED(LedConstants.kLeftLedPort);
  private AddressableLEDBuffer ledBufferL;

  public LedSubsystem() {
    ledBufferR = new AddressableLEDBuffer(LedConstants.kRightLedLength);
    ledR.setLength(ledBufferR.getLength());
    ledR.setData(ledBufferR);
    ledR.start();

    ledBufferL = new AddressableLEDBuffer(LedConstants.kLeftLedLength);
    ledL.setLength(ledBufferL.getLength());
    ledL.setData(ledBufferL);
    ledL.start();
  }

  public LedState getState() {
    return currState;
  }

  private void setState(LedState state) {
    currState = state;
  }

  public void setColor(int r, int g, int b) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      ledBufferR.setRGB(i, r, g, b);
    }
    for (var i = 0; i < ledBufferL.getLength(); i++) {
      ledBufferL.setRGB(i, r, g, b);
    }
  }

  public void setColor(Color color) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      ledBufferR.setLED(i, color);
    }
    for (var i = 0; i < ledBufferL.getLength(); i++) {
      ledBufferL.setLED(i, color);
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
        for (var i = 0; i < ledBufferR.getLength(); i++) {
          ledBufferR.setRGB(i, 250, (int) (Math.random() * 185), 0);
        }
        for (var i = 0; i < ledBufferL.getLength(); i++) {
          ledBufferL.setRGB(i, 250, (int) (Math.random() * 185), 0);
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
    return Set.of(new Measure("State", () -> getState().ordinal()));
  }

  public enum LedState {
    OFF,
    SOLID,
    FLAMING
  }
}
