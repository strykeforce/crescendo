package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class TuningShootCommand extends Command {
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;
  MagazineSubsystem magazineSubsystem;
  IntakeSubsystem intakeSubsystem;
  double lShooterSpeed;
  double rShooterSpeed;
  double magazineSpeed;
  double elbowPos;
  boolean duplicateShooters;

  public TuningShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      double lShooterSpeed,
      double rShooterSpeed,
      double magazineSpeed,
      double elbowPos,
      boolean duplicateShooters) {
    addRequirements(superStructure, magazineSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.lShooterSpeed = lShooterSpeed;
    this.rShooterSpeed = rShooterSpeed;
    this.magazineSpeed = magazineSpeed;
    this.elbowPos = elbowPos;
    this.duplicateShooters = duplicateShooters;
  }

  @Override
  public void initialize() {
    magazineSubsystem.enableLimitSwitches(false);
    magazineSubsystem.setSpeed(magazineSpeed);
    intakeSubsystem.toIntaking();
    superStructure.shoot(
        lShooterSpeed, duplicateShooters ? lShooterSpeed : rShooterSpeed, elbowPos);
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return (curState != RobotStates.SHOOTING || curState != RobotStates.TO_SHOOT);
  }
}
