package frc.robot.subsystems.robotState;

import com.opencsv.CSVReader;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
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
  private ClimbSubsystem climbSubsystem;

  private RobotStates curState = RobotStates.IDLE;
  private RobotStates nextState = RobotStates.IDLE;

  private String[][] lookupTable;

  private Timer shootDelayTimer = new Timer();
  private Timer magazineShootDelayTimer = new Timer();
  private Timer ampStowTimer = new Timer();

  private Alliance allianceColor = Alliance.Blue;

  private boolean safeStow = false;
  private boolean decendClimbAfterTrap;

  private double magazineTuneSpeed = 0.0;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure,
      ClimbSubsystem climbSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.superStructure = superStructure;
    this.climbSubsystem = climbSubsystem;

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
      index = 1;
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
          (int)
              ((distance - RobotStateConstants.kLookupMinDistance)
                      / RobotStateConstants.kDistanceIncrement
                  + 1.0);
      logger.info(
          "Distance: {} | Measured {}", Double.parseDouble(lookupTable[index][0]), distance);
      /*
       * index =
       * (int) (Math.round(distance) - RobotStateConstants.kLookupMinDistance)
       * / RobotStateConstants.kDistanceIncrement;
       */
    }

    shootSolution[0] = Double.parseDouble(lookupTable[index][1]); // Left Shooter
    shootSolution[1] = Double.parseDouble(lookupTable[index][2]); // Right Shooter
    shootSolution[2] = Double.parseDouble(lookupTable[index][3]); // Elbow

    return shootSolution;
  }

  public void setMagazineTune(double speed) {
    magazineTuneSpeed = speed;
  }

  // Control Methods

  public void toSafeIntake() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.safeIntake();
    // magazineSubsystem.toIntaking();
    magazineSubsystem.setEmpty();
    setState(RobotStates.TO_INTAKING);
  }

  public void toIntake() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.intake();
    // magazineSubsystem.toIntaking();
    magazineSubsystem.setEmpty();
    setState(RobotStates.TO_INTAKING);
  }

  public void toAmp() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.amp();
    intakeSubsystem.setPercent(0.0);

    setState(RobotStates.TO_AMP);
  }

  public void startShoot() {
    driveSubsystem.setIsAligningShot(true);

    double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.TO_SHOOT);
  }

  public void toStowSafe() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    magazineSubsystem.setSpeed(0.0);
    superStructure.safeStow();

    setState(RobotStates.TO_STOW);
  }

  public void toStow() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    magazineSubsystem.setSpeed(0.0);
    superStructure.stow();

    setState(RobotStates.TO_STOW);
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
    intakeSubsystem.setPercent(0.0);

    setState(RobotStates.TO_SUBWOOFER);
  }

  public void prepareClimb() {
    climbSubsystem.zero(true);
    climbSubsystem.extendForks();
    superStructure.toPrepClimb();

    setState(RobotStates.PREPPING_CLIMB);
  }

  public void climb() {
    climbSubsystem.trapClimb();

    setState(RobotStates.CLIMBING);
  }

  public void toTrap() {
    superStructure.toFold();

    setState(RobotStates.FOLDING_OUT);
  }

  public void scoreTrap(boolean decend) {
    magazineSubsystem.trap();
    this.decendClimbAfterTrap = decend;
  }

  public void decendClimb() {
    superStructure.toPrepClimb();

    setState(RobotStates.PREPPING_DECEND);
  }

  public void postClimbStow() {
    toStow();
    climbSubsystem.retractForks();
    climbSubsystem.stow();

    setState(RobotStates.TO_STOW);
  }

  // FIXME
  public void releaseGamePiece() {
    if (curState == RobotStates.TO_PODIUM) {
      superStructure.podiumShoot();

      magazineShootDelayTimer.stop();
      magazineShootDelayTimer.reset();
      magazineShootDelayTimer.start();

      setState(RobotStates.PODIUM_SHOOTING);
    } else {
      safeStow = curState == RobotStates.AMP;
      magazineSubsystem.toReleaseGamePiece();
      setState(RobotStates.RELEASE);
    }
  }

  public void toTune() {
    setState(RobotStates.TO_TUNE);
    superStructure.shootTune();
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
          setState(RobotStates.INTAKING);
        }
        break;

      case INTAKING:
        if (intakeSubsystem.getState() == IntakeState.HAS_PIECE
            && (magazineSubsystem.getState() != MagazineStates.INTAKING
                && magazineSubsystem.getState() != MagazineStates.REVERSING)) {
          magazineSubsystem.toIntaking();
        }
        if (magazineSubsystem.hasPiece()) {
          // Magazine stops running upon detecting a game piece

          intakeSubsystem.setPercent(0);

          toStow();
        }
        break;

      case TO_AMP:
        if (superStructure.isFinished()) {
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

          toStowSafe(); // FIXME: call stow() and possibly wait for timeout
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
          toIntake();
          magazineSubsystem.setEmpty();
        }

        break;

      case TO_PODIUM:
        if (magazineSubsystem.getState() == MagazineStates.SPEEDUP) {
          superStructure.slowWheelSpin();
        }
        // if (superStructure.isFinished() && magazineSubsystem.getState() ==
        // MagazineStates.SHOOT)
        // {
        // superStructure.podiumShoot();

        // magazineShootDelayTimer.stop();
        // magazineShootDelayTimer.reset();
        // magazineShootDelayTimer.start();

        // setState(RobotStates.PODIUM_SHOOTING);
        // }
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

      case RELEASE:
        if (magazineSubsystem.getState() != MagazineStates.RELEASE) {
          if (safeStow) toSafeIntake();
          else toIntake();
          safeStow = false;
        }
        break;
      case TO_TUNE:
        if (superStructure.isFinished()) {
          magazineSubsystem.toEmptying(magazineTuneSpeed);

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }
        break;
      case PREPPING_CLIMB:
        if (climbSubsystem.isFinished() && superStructure.isFinished()) {
          setState(RobotStates.CLIMB_PREPPED);
        }
        break;
      case CLIMB_PREPPED:
        break;

      case CLIMBING:
        if (climbSubsystem.isFinished()) {
          setState(RobotStates.CLIMBED);
        }
        break;
      case CLIMBED:
        break;
      case FOLDING_OUT:
        if (superStructure.isFinished()) {
          superStructure.toTrap();
          climbSubsystem.extendTrapBar();

          setState(RobotStates.TO_TRAP);
        }
        break;
      case TO_TRAP:
        if (superStructure.isFinished()) {
          setState(RobotStates.TRAP);
        }
        break;
      case TRAP:
        if (!magazineSubsystem.hasPiece()) {
          climbSubsystem.retractTrapBar();
          superStructure.toFold();
          setState(RobotStates.FOLDING_IN);
        }
        break;
      case FOLDING_IN:
        if (superStructure.isFinished()) {
          superStructure.toPrepClimb();

          if (decendClimbAfterTrap) {
            setState(RobotStates.PREPPING_DECEND);
          }
        }
        break;
      case PREPPING_DECEND:
        if (superStructure.isFinished()) {
          climbSubsystem.descend();

          setState(RobotStates.DESCENDING);
        }
        break;
      case DESCENDING:
        if (climbSubsystem.isFinished()) {
          setState(RobotStates.POST_CLIMB);
        }
        break;
      case POST_CLIMB:
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
    TO_SUBWOOFER,
    RELEASE,
    TO_TUNE,
    PREPPING_CLIMB,
    CLIMB_PREPPED,
    TO_TRAP,
    TRAP,
    FOLDING_OUT,
    FOLDING_IN,
    DESCENDING,
    POST_CLIMB,
    PREPPING_DECEND,
    CLIMBING,
    CLIMBED
  }
}
