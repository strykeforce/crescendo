package frc.robot.subsystems.robotState;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotStateSubsystem extends MeasurableSubsystem {

  public Alliance getAllianceColor() {
    return Alliance.Red;
    // FIXME
  }

  // Private Variables
  VisionSubsystem visionSubsystem;
  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElbowSubsystem elbowSubsystem;
  SuperStructure superStructure;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem, SuperStructure superStructure) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.superStructure = superStructure;
  }

  // Getter/Setter Methods

  // Helper Methods

  // Periodic

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
      return null;
  }

  // State
  public enum RobotStates {}
}
