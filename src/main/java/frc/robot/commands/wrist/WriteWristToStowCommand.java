package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WriteWristToStowCommand extends InstantCommand {
  private WristSubsystem wristSubsystem;

  public WriteWristToStowCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
  }

  @Override
  public void initialize() {
    wristSubsystem.forceWristPos(SuperStructureConstants.kWristStowSetPoint);
  }
}
