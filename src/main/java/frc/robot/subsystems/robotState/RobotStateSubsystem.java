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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  // Private Variables
  private Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private RobotStates curState;

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

  // For convinient logging
  private void setState(RobotStates newState) {
    logger.info("{} -> {}", curState, newState);
    curState = newState;
  }

  // Helper Methods
  public void toIntake() {
    superStructure.intake();
    intakeSubsystem.toIntaking();
    magazineSubsystem.toIntaking();

    setState(RobotStates.TO_INTAKING);
  }

  public void toAmp() {
    superStructure.amp();

    setState(RobotStates.TO_AMP);
  }

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case IDLE:
        break;

      case TO_INTAKING:
        if (superStructure.isFinished()) {

          setState(RobotStates.INTAKING);
        }
        break;

      case INTAKING:
        if (magazineSubsystem.hasPiece()) {
          // Magazine stops running upon detecting a game piece
          intakeSubsystem.setPercent(0);
          setState(RobotStates.IDLE);
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
          setState(RobotStates.AMP);
        }
        break;
      case AMP:
        if (!magazineSubsystem.hasPiece()) {
          setState(RobotStates.IDLE);
        }
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
    IDLE,
    INTAKING,
    TO_INTAKING,
    TO_AMP,
    AMP
  }
}
