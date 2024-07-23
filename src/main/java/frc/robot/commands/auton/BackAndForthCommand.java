package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.SetHoloContKPCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class BackAndForthCommand extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;
  RobotStateSubsystem robotStateSubsystem;
  SuperStructure superStructure;
  MagazineSubsystem magazineSubsystem;
  IntakeSubsystem intakeSubsystem;

  public BackAndForthCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      String pathOne,
      String pathTwo) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addCommands(
        new ResetGyroCommand(driveSubsystem),
        new SetHoloContKPCommand(driveSubsystem, 0.5),
        new StowCommand(robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem),
        firstPath,
        secondPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    hasGenerated = true;
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
