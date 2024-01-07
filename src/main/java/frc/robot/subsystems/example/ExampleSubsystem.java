package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ExampleConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;

public class ExampleSubsystem extends SubsystemBase implements ClosedLoopPosSubsystem {
  // Private Variables
  private final ExampleIO io;
  private double setpoint = 0.0;
  private ExampleState curState = ExampleState.INIT;

  // Constructor
  public ExampleSubsystem(ExampleIO io) {
    this.io = io;
    zero();
  }

  // Getter/Setter Methods
  public ExampleState getState() {
    return curState;
  }

  @Override
  public double getPosition() {
    return setpoint;
  }

  @Override
  public void setPosition(double position) {
    io.setPosition(position);
    setpoint = position;
  }

  // Helper Methods
  @Override
  public void zero() {
    io.zero();
    curState = ExampleState.ZEROED;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(setpoint) <= ExampleConstants.kCloseEnough;
  }

  // Periodic Function
  @Override
  public void periodic() {
    super.periodic();
  }

  // Grapher

  // State Enum
  public enum ExampleState {
    INIT,
    ZEROED
  }
}
