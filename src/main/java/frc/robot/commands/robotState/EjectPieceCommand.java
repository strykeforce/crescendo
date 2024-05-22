package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class EjectPieceCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public EjectPieceCommand(
      RobotStateSubsystem robotStateSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure) {
    addRequirements(magazineSubsystem, superStructure);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toEjecting();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() != RobotStates.EJECTING;
  }
}
