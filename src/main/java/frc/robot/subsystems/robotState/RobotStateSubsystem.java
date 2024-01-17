package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  // Private Variables
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private RobotStates curState, nextState;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElbowSubsystem elbowSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
  }

  // Getter/Setter Methods
  public Alliance getAllianceColor() {
    return Alliance.Red;
    // FIXME
  }

  // Helper Methods
  public void toIntake() {
    intakeSubsystem.toIntaking();
    magazineSubsystem.toIntaking();
    superStructure.intake();

    curState = RobotStates.INTAKING;
  }

  public void toAmp() {
    superStructure.amp();

    curState = RobotStates.TO_AMP;
  }

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case IDLE:
        break;

      case INTAKING:
        if (magazineSubsystem.hasPiece()) {
          curState = RobotStates.IDLE;
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
          curState = RobotStates.AMP;
        }
        break;
      case AMP:
        break;

      default:
        break;
    }
  }

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return null;
  }

  // State
  public enum RobotStates {
    INTAKING,
    IDLE,
    TO_AMP,
    AMP
  }
}
