package frc.robot.commands.superStructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superStructure.SuperStructure;

public class SpinUpWheelsCommand extends InstantCommand {
  private SuperStructure superStructure;
  private double lSpeed, rSpeed;

  public SpinUpWheelsCommand(SuperStructure superStructure, double lSpeed, double rSpeed) {
    this.superStructure = superStructure;
    this.lSpeed = lSpeed;
    this.rSpeed = rSpeed;
  }

  @Override
  public void initialize() {
    superStructure.spinUpWheels(lSpeed, rSpeed);
  }
}
