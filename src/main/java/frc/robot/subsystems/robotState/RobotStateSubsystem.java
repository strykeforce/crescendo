package frc.robot.subsystems.robotState;

import com.opencsv.CSVReader;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem.MagazineStates;
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
  private IntakeSubsystem intakeSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private SuperStructure superStructure;

  private RobotStates curState = RobotStates.IDLE;
  private RobotStates nextState = RobotStates.IDLE;

  private String[][] lookupTable;

  private Timer shootDelayTimer = new Timer();
  private Timer magazineShootDelayTimer = new Timer();

  private Timer ampStowTimer = new Timer();

  private Alliance allianceColor = Alliance.Blue;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.superStructure = superStructure;

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

  public void setAllianceColor(Alliance alliance) {
    allianceColor = alliance;
    logger.info("Change color to: {}", allianceColor);
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public boolean hasNote() {
    return magazineSubsystem.hasPiece() || intakeSubsystem.isBeamBroken();
  }

  // Helper Methods
  private void toNextState() {
    setState(nextState);

    nextState = RobotStates.STOW;
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

  public void startShoot() {
    driveSubsystem.setIsAligningShot(true);

    double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.TO_SHOOT);
  }

  public void toStow() {
    driveSubsystem.setIsAligningShot(false);
    setState(RobotStates.TO_STOW);
    intakeSubsystem.setPercent(0.0);
    magazineSubsystem.setSpeed(0.0);
    superStructure.stow();
  }

  public void toPreparePodium() {
    driveSubsystem.setIsAligningShot(false);
    magazineSubsystem.preparePodium();
    intakeSubsystem.setPercent(0.0);
    superStructure.preparePodium();

    setState(RobotStates.TO_PODIUM);
  }

  public void toSubwoofer() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    superStructure.subwoofer();

    setState(RobotStates.TO_SUBWOOFER);
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
          // Magazine stops running upon detecting a game piece
          intakeSubsystem.setPercent(0);

          toStow();
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
          magazineSubsystem.toEmptying();
          setState(RobotStates.AMP);
        }
        break;

      case AMP:
        if (!magazineSubsystem.hasPiece()) {
          ampStowTimer.stop();
          ampStowTimer.reset();
          ampStowTimer.start();
        }

        if (!magazineSubsystem.hasPiece()
            && ampStowTimer.hasElapsed(RobotStateConstants.kTimeToStowPostAmp)) {
          ampStowTimer.stop();
          ampStowTimer.reset();
          ampStowTimer.start();

          toStow(); // FIXME: call stow() and possibly wait for timeout
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

          superStructure.stopShoot();
          toStow();
        }

        break;

      case TO_PODIUM:
        if (magazineSubsystem.getState() == MagazineStates.SPEEDUP) {
          superStructure.stopShoot();
        }
        if (superStructure.isFinished() && magazineSubsystem.getState() == MagazineStates.SHOOT) {
          superStructure.podiumShoot();

          magazineShootDelayTimer.stop();
          magazineShootDelayTimer.reset();
          magazineShootDelayTimer.start();

          setState(RobotStates.PODIUM_SHOOTING);
        }
        break;

      case PODIUM_SHOOTING:
        if (magazineShootDelayTimer.hasElapsed(ShooterConstants.kShootTime)) {
          magazineShootDelayTimer.stop();

          superStructure.stopPodiumShoot();
          toStow();
        }
        break;

      case TO_SUBWOOFER:
        if (superStructure.isFinished()) {
          magazineSubsystem.toEmptying();

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }
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

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  // State
  public enum RobotStates {
    IDLE,
    TO_INTAKING,
    INTAKING,
    TO_AMP,
    AMP,
    TO_STOW,
    STOW,
    TO_SHOOT,
    SHOOTING,
    TO_PODIUM,
    PODIUM_SHOOTING,
    TO_SUBWOOFER
  }
}
