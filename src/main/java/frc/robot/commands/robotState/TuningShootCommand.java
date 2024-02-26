package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TuningShootCommand extends InstantCommand {
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;
  MagazineSubsystem magazineSubsystem;
  IntakeSubsystem intakeSubsystem;
  DoubleSupplier lShooterSpeed;
  DoubleSupplier rShooterSpeed;
  DoubleSupplier magazineSpeed;
  DoubleSupplier elbowPos;
  BooleanSupplier duplicateShooters;

  public TuningShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      DoubleSupplier lShooterSpeed,
      DoubleSupplier rShooterSpeed,
      DoubleSupplier magazineSpeed,
      DoubleSupplier elbowPos,
      BooleanSupplier duplicateShooters) {
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
    superStructure.saveSetpoint(
        -lShooterSpeed.getAsDouble(),
        duplicateShooters.getAsBoolean()
            ? lShooterSpeed.getAsDouble()
            : rShooterSpeed.getAsDouble(),
        elbowPos.getAsDouble());
    robotStateSubsystem.setMagazineTune(magazineSpeed.getAsDouble());
  }
}
