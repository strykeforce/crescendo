package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;

public class TrapBarIOServo implements TrapBarIO {
  private Logger logger;
  private Servo leftTrap;
  private Servo rightTrap;

  public TrapBarIOServo() {
    leftTrap = new Servo(ClimbConstants.kLeftTrapBarId);
    rightTrap = new Servo(ClimbConstants.kRightTrapBarId);
  }

  @Override
  public void updateInputs(TrapBarIOInputs inputs) {
    inputs.leftPos = leftTrap.getPosition();
    inputs.rightPos = rightTrap.getPosition();
  }

  @Override
  public void setPosition(double leftPos, double rightPos) {
    leftTrap.set(leftPos);
    rightTrap.set(rightPos);
  }
}
