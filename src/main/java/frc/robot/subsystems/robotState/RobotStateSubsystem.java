package frc.robot.subsystems.robotState;

import com.opencsv.CSVReader;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
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
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  // Private Variables
  private Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private SuperStructure superStructure;
  private ShooterSubsystem shooterSubsystem;

  private RobotStates curState = RobotStates.STOW;
  private RobotStates nextState = RobotStates.STOW;

  private boolean hasNote = false;

  private String[][] lookupTable;

  private Timer shootDelayTimer = new Timer();

  private boolean atPodium = false;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure,
      ShooterSubsystem shooterSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.superStructure = superStructure;
    this.shooterSubsystem = shooterSubsystem;

    parseLookupTable();
  }

  // Getter/Setter Methods
  public RobotStates getState() {
    return curState;
  }

  private void setState(RobotStates robotState) {
    if (this.curState != robotState) {
      logger.info("{} -> {}", this.curState, robotState);
      this.curState = robotState;
    }
  }

  public Alliance getAllianceColor() {
    return Alliance.Red;
    // FIXME
  }

  public boolean hasNote() {
    return magazineSubsystem.hasPiece() || intakeSubsystem.isBeamBroken();
  }

  // Helper Methods
  public void toIntake() {
    superStructure.intake();
    intakeSubsystem.toIntaking();
    magazineSubsystem.toIntaking();
    driveSubsystem.setIsAligningShot(false);

    setState(RobotStates.TO_INTAKING);
  }

  public void toAmp() {
    superStructure.amp();
    driveSubsystem.setIsAligningShot(false);

    setState(RobotStates.TO_AMP);
  }

  private void toNextState() {
    setState(nextState);

    nextState = RobotStates.STOW;
  }

  private void toPodium() {
    superStructure.podium();
    driveSubsystem.setIsAligningShot(false);
    magazineSubsystem.preparePodium();
    shooterSubsystem.preparePodium();
  }

  // Order of Columns: dist meters, left shoot, right shoot, elbow, time of flight
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

    shootSolution[0] = Double.parseDouble(lookupTable[index][1]); // Left Shooter
    shootSolution[1] = Double.parseDouble(lookupTable[index][2]); // Right Shooter
    shootSolution[2] = Double.parseDouble(lookupTable[index][3]); // Elbow

    return shootSolution;
  }

  // Control Methods
  public void startShoot() {
    driveSubsystem.setIsAligningShot(true);
    
    double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());

    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    magazineSubsystem.setSpeed(0.0);
    nextState = RobotStates.TO_SHOOT;
  } 




  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case STOW:
        toNextState();

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
          setState(RobotStates.STOW);
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
          setState(RobotStates.AMP);
        }
        break;
      case AMP:
        if (!magazineSubsystem.hasPiece()) {
          setState(RobotStates.IDLE); // FIXME: call stow() and possibly wait for timeout
        }
        break;

      case TO_SHOOT:
        double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());
        superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

        if (driveSubsystem.isDriveStill()
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
          // FIXME: stop shooter

          setState(RobotStates.STOW); // FIMXE: actually stow
        }

        break;

      case TO_PODIUM:
        
        break;

      case PODIUM :
        break;

      default:
        break;
    }

    org.littletonrobotics.junction.Logger.recordOutput("Robot State", curState.ordinal());
  }

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  // State
  public enum RobotStates {
    IDLE,
    INTAKING,
    TO_INTAKING,
    TO_AMP,
    AMP,
    STOW,
    TO_SHOOT,
    SHOOTING,
    TO_PODIUM,
    PODIUM
  }
}
