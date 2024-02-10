package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TuningShootCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;
  MagazineSubsystem magazineSubsystem;
  RobotContainer robotContainer;

  public TuningShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      RobotContainer robotContainer) {
    addRequirements(superStructure, magazineSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    magazineSubsystem.enableLimitSwitches(false);
    magazineSubsystem.setSpeed(robotContainer.magazineSpeed.getDouble(0.0));
    superStructure.shoot(
        robotContainer.lShooterSpeed.getDouble(0.0),
        robotContainer.duplicateShooters.getBoolean(true)
            ? robotContainer.lShooterSpeed.getDouble(0.0)
            : robotContainer.rShooterSpeed.getDouble(0.0),
        robotContainer.elbowPos.getDouble(0.0));
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return (curState != RobotStates.SHOOTING || curState != RobotStates.TO_SHOOT);
  }
}
