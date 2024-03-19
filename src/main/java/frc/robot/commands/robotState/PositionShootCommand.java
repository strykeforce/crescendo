package frc.robot.commands.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.superStructure.SuperStructure;

public class PositionShootCommand extends Command implements AutoCommandInterface {
  RobotStateSubsystem robotStateSubsystem;
  Pose2d shootPos;
  Pose2d generatedPos;
  DriveSubsystem driveSubsystem;
  boolean triedFlip = false;

  public PositionShootCommand(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      DriveSubsystem driveSubsystem,
      Pose2d shootPos) {
    addRequirements(superStructure, magazineSubsystem, intakeSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.shootPos = shootPos;
    this.generatedPos = shootPos;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.startShootKnownPos(generatedPos);
  }

  @Override
  public boolean hasGenerated() {
    return triedFlip;
  }

  @Override
  public void generateTrajectory() {
    triedFlip = true;
    generatedPos = driveSubsystem.apply(shootPos);
  }

  @Override
  public boolean isFinished() {
    RobotStates curState = robotStateSubsystem.getState();

    return (curState != RobotStates.SHOOTING && curState != RobotStates.TO_SHOOT);
  }
}
