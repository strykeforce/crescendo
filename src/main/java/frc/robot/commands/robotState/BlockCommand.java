package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.superStructure.SuperStructure.SuperStructureStates;

public class BlockCommand extends Command {
  private SuperStructure superStructure;

  public BlockCommand(SuperStructure superStructure) {
    addRequirements(superStructure);
    this.superStructure = superStructure;
  }

  @Override
  public void initialize() {
    superStructure.block();
  }

  @Override
  public boolean isFinished() {
    return superStructure.getState() != SuperStructureStates.TRANSFER;
  }
}
