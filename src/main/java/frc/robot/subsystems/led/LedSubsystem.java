package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LedConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LedSubsystem extends MeasurableSubsystem {

  private LedState currState = LedState.OFF;

  private AddressableLED ledR = new AddressableLED(LedConstants.kRightLedPort);
  private AddressableLEDBuffer ledBufferR = new AddressableLEDBuffer(LedConstants.kRightLedLength);
  private int candyIterator = 0;
  private int loopCounter = 0;

  //   private AddressableLED ledL = new AddressableLED(LedConstants.kLeftLedPort);
  //   private AddressableLEDBuffer ledBufferL;

  private Logger logger = LoggerFactory.getLogger(this.getClass());

  public LedSubsystem() {
    ledR.setLength(ledBufferR.getLength());
    ledR.setData(ledBufferR);
    ledR.start();

    // ledBufferL = new AddressableLEDBuffer(LedConstants.kLeftLedLength);
    // ledL.setLength(ledBufferL.getLength());
    // ledL.setData(ledBufferL);
    // ledL.start();

    logger.info("created LedSubsystem");
  }

  public LedState getState() {
    return currState;
  }

  private void setState(LedState state) {
    currState = state;
  }

  // this has no data setting! only sets the buffer!
  private void setLED(int i, int r, int g, int b) {
      ledBufferR.setRGB(i, g, r, b);
  }

  // this has no data setting! only sets the buffer!
  private void setLED(int i, Color color) {
      ledBufferR.setRGB(
          i, (int) (color.green * 255.0), (int) (color.red * 255.0), (int) (color.blue * 255.0));
  }

  public void setColor(int r, int g, int b) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, g, r, b);
    }
    // for (var i = 0; i < ledBufferL.getLength(); i++) {
    //   ledBufferL.setRGB(i, r, g, b);
    // }
    ledR.setData(ledBufferR);
    // ledL.setData(ledBufferL);
    logger.info("set color to: {}, {}, {}", r, g, b);
  }

  public void setColor(Color color) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, color);
    }
    // for (var i = 0; i < ledBufferL.getLength(); i++) {
    //   ledBufferL.setLED(i, color);
    // }
    ledR.setData(ledBufferR);
    // ledL.setData(ledBufferL);
    logger.info("set color to: {}, {}, {}", color.red, color.green, color.blue);
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

  public void setCandy() {
    setState(LedState.CANDY);
    candyIterator = 0;
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
          setLED(i, 250, (int) (Math.random() * 185), 0);
          ledR.setData(ledBufferR);
        }
        // for (var i = 0; i < ledBufferL.getLength(); i++) {
        //   ledBu249,172,252tData(ledBufferL);

        break;
      case SOLID:
        break;
      case CANDY:
        for (var i = 0; i < ledBufferR.getLength(); i++) {
          setLED(i, LedConstants.candy[(i + candyIterator) % 5]);
        }
        ledR.setData(ledBufferR);
        if (loopCounter >= LedConstants.kLoopCounterCandy) {
          if (candyIterator >= LedConstants.candy.length - 1) {
            candyIterator = 0;
          } else {
            candyIterator++;
          }
          loopCounter = 0;
        } else {
          loopCounter++;
        }
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
    FLAMING,
    CANDY
  }
}
