package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.ResetOdomCommand;
import frc.robot.commands.drive.setAngleOffsetCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class DoNothingCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private RobotStateSubsystem robotStateSubsystem;
  private DriveSubsystem driveSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private ElbowSubsystem elbowSubsystem;

  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;

  public DoNothingCommand(
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      ElbowSubsystem elbowSubsystem,
      Pose2d start) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.elbowSubsystem = elbowSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new ZeroElbowCommand(elbowSubsystem), // zero elbow
            new ResetGyroCommand(driveSubsystem)), // reset gyro
        new setAngleOffsetCommand(driveSubsystem, start.getRotation().getDegrees()),
        new ResetOdomCommand(driveSubsystem, start),
        new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem)); // shoot
  }

  @Override
  public void generateTrajectory() {
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  @Override
  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
