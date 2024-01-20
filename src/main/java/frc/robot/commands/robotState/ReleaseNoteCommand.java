package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class ReleaseNoteCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  MagazineSubsystem magazineSubsystem;
  SuperStructure superStructure;

  public ReleaseNoteCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem) {
    addRequirements(superStructure, magazineSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.releaseGamePiece();
  }

  @Override
  public boolean isFinished() {
    return !robotStateSubsystem.hasNote();
  }
}
