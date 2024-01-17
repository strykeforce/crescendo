package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotStateSubsystem {

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

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elbowSubsystem = elbowSubsystem;
  }

  // Getter/Setter Methods
  public boolean hasGamePiece() {
    // FIXME
    return true;
  }
  // Helper Methods

  public void shoot() {
    // FIXME
  }

  // Periodic

  // Grapher

  // State
  public enum RobotStates {}
}
