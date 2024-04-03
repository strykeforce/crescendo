package frc.robot.commands.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class PrepShooterCommand extends InstantCommand {
  private SuperStructure superStructure;
  private RobotStateSubsystem robotStateSubsystem;
  private Pose2d shootPos;

  public PrepShooterCommand(
      SuperStructure superStructure, RobotStateSubsystem robotStateSubsystem, Pose2d shootPos) {
    addRequirements(superStructure);
    this.shootPos = shootPos;
    this.superStructure = superStructure;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.hasNote()) robotStateSubsystem.spinUpShotSolution(shootPos);
  }
}
