package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.subsystems.auto.AutoCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;

public class DefaultAutoCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private ElbowSubsystem elbowSubsystem;

  private DriveAutonCommand defaultPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Blue;

  public DefaultAutoCommand(
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      ElbowSubsystem elbowSubsystem,
      String path) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.elbowSubsystem = elbowSubsystem;

    this.defaultPath = new DriveAutonCommand(driveSubsystem, path, true, true);

    addCommands(
        new ParallelCommandGroup(
            new ZeroElbowCommand(elbowSubsystem) // set gyro offset
            ), // shoot
        defaultPath);
  }

  @Override
  public void generateTrajectory() {
    defaultPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  @Override
  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
