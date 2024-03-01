package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;

public class ClimbRatchetIOServo implements ClimbRatchetIO {
  private Logger logger;
  private Servo leftRatchet;
  private Servo rightRatchet;

  public ClimbRatchetIOServo() {
    leftRatchet = new Servo(ClimbConstants.kLeftRatchetId);
    rightRatchet = new Servo(ClimbConstants.kRightRatchetId);
  }

  @Override
  public void updateInputs(ClimbRatchetIOInputs inputs) {
    inputs.leftPos = leftRatchet.getPosition();
    inputs.rightPos = rightRatchet.getPosition();
  }

  @Override
  public void setPosition(double leftPos, double rightPos) {
    leftRatchet.set(leftPos);
    rightRatchet.set(rightPos);
  }
}
