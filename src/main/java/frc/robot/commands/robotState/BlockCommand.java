package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.superStructure.SuperStructure.SuperStructureStates;

public class BlockCommand extends Command {
  private SuperStructure superStructure;
  private RobotStateSubsystem robotStateSubsystem;

  public BlockCommand(
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.superStructure = superStructure;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    if (superStructure.getState() == SuperStructureStates.BLOCK) {
      robotStateSubsystem.postClimbStow();
    } else {
      robotStateSubsystem.blockShot();
    }
  }

  @Override
  public boolean isFinished() {
    return superStructure.getState() != SuperStructureStates.TRANSFER;
  }
}
