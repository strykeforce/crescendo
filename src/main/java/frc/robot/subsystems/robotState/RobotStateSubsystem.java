package frc.robot.subsystems.robotState;

import com.opencsv.CSVReader;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.MagazineConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.FileReader;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  // Private Variables
  private Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private SuperStructure superStructure;

  private RobotStates curState = RobotStates.STOW;
  private RobotStates nextState = RobotStates.STOW;

  private boolean hasNote = false;

  private String[][] lookupTable;

  private Timer shootDelayTimer = new Timer();

  private Alliance allianceColor = Alliance.Blue;

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
    this.magazineSubsystem = magazineSubsystem;
    this.superStructure = superStructure;

    parseLookupTable();
  }

  // Getter/Setter Methods
  public RobotStates getState() {
    return curState;
  }

  public boolean hasNote() {
    return hasNote;
  }

  public void setState(RobotStates robotState) {
    if (this.curState != robotState) {
      logger.info("{} -> {}", this.curState, robotState);
      this.curState = robotState;
    }
  }

  public void setAllianceColor(Alliance alliance) {
    allianceColor = alliance;
    logger.info("Change color to: {}", allianceColor);
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  // Helper Methods
  public void toIntake() {

    driveSubsystem.setIsAligningShot(false);
    superStructure.intake();
    intakeSubsystem.toIntaking();
    magazineSubsystem.toIntaking();

    setState(RobotStates.TO_INTAKING);
  }

  public void toAmp() {

    driveSubsystem.setIsAligningShot(false);
    superStructure.amp();

    setState(RobotStates.TO_AMP);
  }

  private void toNextState() {
    setState(nextState);

    nextState = RobotStates.STOW;
  }

  private void parseLookupTable() {
    try {
      CSVReader csvReader = new CSVReader(new FileReader(RobotStateConstants.kLookupTablePath));

      List<String[]> list = csvReader.readAll();
      String[][] strArr = new String[list.size()][];

      lookupTable = list.toArray(strArr);
    } catch (Exception exception) {

    }
  }

  private double[] getShootSolution(double distance) {
    double[] shootSolution = new double[3];
    int index;

    if (distance < RobotStateConstants.kLookupMinDistance) {
      index = 0;
      logger.warn(
          "Distance {} is less than min distance in table {}",
          distance,
          RobotStateConstants.kLookupMinDistance);
    } else if (distance > RobotStateConstants.kLookupMaxDistance) {
      index = lookupTable.length - 1;
      logger.warn(
          "Distance {} is more than max distance in table {}",
          distance,
          RobotStateConstants.kLookupMaxDistance);
    } else {
      index =
          (int) (Math.round(distance) - RobotStateConstants.kLookupMinDistance)
              / RobotStateConstants.kDistanceIncrement;
    }

    shootSolution[0] = Double.parseDouble(lookupTable[index][1]);
    shootSolution[1] = Double.parseDouble(lookupTable[index][2]);
    shootSolution[2] = Double.parseDouble(lookupTable[index][3]);

    return shootSolution;
  }

  // Control Methods
  public void shoot() {
    driveSubsystem.setIsAligningShot(true);

    double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.SHOOT_ALIGN);
  }

  public void toStow() {

    driveSubsystem.setIsAligningShot(false);
    setState(RobotStates.TO_STOW);
    intakeSubsystem.setPercent(0.0);
    magazineSubsystem.setSpeed(0.0);
    superStructure.stow();
  }

  // FIXME
  public void releaseGamePiece() {
    magazineSubsystem.toEmptying();
  }

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case TO_STOW:
        if (superStructure.isFinished()) {
          setState(RobotStates.STOW);
        }

        break;
      case STOW:
        break;

      case TO_INTAKING:
        if (superStructure.isFinished()) {
          intakeSubsystem.toIntaking();
          magazineSubsystem.toIntaking();
          setState(RobotStates.INTAKING);
        }
        break;

      case INTAKING:
        if (magazineSubsystem.hasPiece()) {
          hasNote = true;
          // Magazine stops running upon detecting a game piece
          intakeSubsystem.setPercent(0);

          setState(RobotStates.IDLE);
          // FIXME should this be Stow?
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
          setState(RobotStates.AMP);
        }
        break;
      case AMP:
        if (!magazineSubsystem.hasPiece()) {
          hasNote = false;
          setState(RobotStates.IDLE);
          // FIXME should this be Stow or intaking?
        }
        break;

      case SHOOT_ALIGN:
        double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());
        superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

        if (driveSubsystem.isVelocityStable()
            && driveSubsystem.isPointingAtGoal()
            && superStructure.isFinished()) {
            
            magazineSubsystem.toEmptying();

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }

        break;

      case SHOOTING:
        if (shootDelayTimer.hasElapsed(ShooterConstants.kShootTime)) {
          shootDelayTimer.stop();
          driveSubsystem.setIsAligningShot(false);
          magazineSubsystem.setSpeed(0);
          hasNote = false;
          toIntake();
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

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  // State
  public enum RobotStates {
    IDLE,
    INTAKING,
    TO_INTAKING,
    TO_AMP,
    AMP,
    STOW,
    SHOOT_ALIGN,
    SHOOTING,
    TO_STOW,
    SHOOT_AIM
  }
}
