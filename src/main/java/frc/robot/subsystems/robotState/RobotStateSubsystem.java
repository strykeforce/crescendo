package frc.robot.subsystems.robotState;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.opencsv.CSVReader;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.MagazineConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.LedSubsystem.LedState;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem.MagazineStates;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.superStructure.SuperStructure.SuperStructureStates;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.FileReader;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import net.jafama.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  // Private Variables
  private Logger logger = LoggerFactory.getLogger(this.getClass());

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private SuperStructure superStructure;
  private ClimbSubsystem climbSubsystem;
  private LedSubsystem ledSubsystem;
  private static CANBus canBus;

  private RobotStates curState = RobotStates.IDLE;
  private RobotStates nextState = RobotStates.IDLE;

  private String[][] shootingLookupTable;
  private String[][] feedingLookupTable;

  private Timer shootDelayTimer = new Timer();
  private Timer magazineShootDelayTimer = new Timer();
  private Timer ampStowTimer = new Timer();
  private Timer startShootDelay = new Timer();
  private Timer climbTrapTimer = new Timer();
  private Timer scoreTrapTimer = new Timer();
  private boolean hasDelayed = false;
  private double shootDelay = 0.0;
  private boolean hasShootBeamUnbroken = false;

  private Alliance allianceColor = Alliance.Blue;

  private boolean safeStow = false;
  private boolean decendClimbAfterTrap = false;
  private boolean continueToTrap = false;
  private boolean usingDistance = false;
  private boolean isAuto = false;
  private boolean shootKnownPos = false;
  private Pose2d shootPos;
  private double grabbedShotDistance = 0.0;
  private double magazineTuneSpeed = 0.0;

  private RobotStates desiredState = RobotStates.STOW;
  private int curShot = 1;

  private double elbowOffset = RobotStateConstants.kElbowShootOffset;

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      MagazineSubsystem magazineSubsystem,
      SuperStructure superStructure,
      ClimbSubsystem climbSubsystem,
      LedSubsystem ledSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.superStructure = superStructure;
    this.climbSubsystem = climbSubsystem;
    this.ledSubsystem = ledSubsystem;

    this.canBus = new CANBus();
    grabElbowOffsetPreferences();

    shootingLookupTable = parseLookupTable(RobotStateConstants.kShootingLookupTablePath);
    feedingLookupTable = parseLookupTable(RobotStateConstants.kFeedingLookupTablePath);
  }

  // Getter/Setter Methods
  public RobotStates getState() {
    return curState;
  }

  public boolean isCANivoreConnected() {
    CANBusStatus status = canBus.getStatus(RobotStateConstants.kcanivoreString);
    return status.Status.isOK();
  }

  private void setState(RobotStates robotState) {
    if (this.curState != robotState) {
      logger.info("{} -> {}", this.curState, robotState);
      this.curState = robotState;
    }
  }

  public void grabElbowOffsetPreferences() {
    elbowOffset =
        Preferences.getDouble(
            RobotStateConstants.kElbowPreferencesKey, RobotStateConstants.kElbowShootOffset);
  }

  public double getElbowOffset() {
    return elbowOffset;
  }

  public void setElbowOffsetPreferences(double offset) {
    Preferences.setDouble(RobotStateConstants.kElbowPreferencesKey, offset);
    elbowOffset = offset;
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

  public boolean intakeHasNote() {
    return intakeSubsystem.hasNote();
  }

  public boolean magazineHasNote() {
    return magazineSubsystem.hasPiece();
  }

  // Helper Methods
  private void toNextState() {
    setState(nextState);

    nextState = RobotStates.STOW;
  }

  // Order of Columns: dist meters, left shoot, right shoot, elbow, time of flight
  private String[][] parseLookupTable(String path) {
    String[][] lookupTable;
    List<String[]> list = new LinkedList<>();

    try {
      CSVReader csvReader = new CSVReader(new FileReader(path));
      list = csvReader.readAll();
    } catch (Exception e) {
      logger.warn("Failed to read lookup table at {} due to {}", path, e);
    }

    String[][] strArr = new String[list.size()][];
    lookupTable = list.toArray(strArr);

    return lookupTable;
  }

  private double[] getShootSolution(double distance, String[][] lookupTable) {
    double[] shootSolution = new double[3];
    int index;
    distance += RobotStateConstants.kDistanceOffset;
    grabbedShotDistance = distance;
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

    logger.info("Left Shooter: {}", lookupTable[index][1]);

    shootSolution[0] = Double.parseDouble(lookupTable[index][1]); // Left Shooter
    shootSolution[1] = Double.parseDouble(lookupTable[index][2]); // Right Shooter
    shootSolution[2] = Double.parseDouble(lookupTable[index][3]) + elbowOffset; // Elbow

    return shootSolution;
  }

  public boolean getIsAuto() {
    return isAuto;
  }

  public void setMagazineTune(double speed) {
    magazineTuneSpeed = speed;
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
    superStructure.setIsAuto(isAuto);
    intakeSubsystem.setIsAuto(isAuto);
  }

  // Control Methods

  public void toSafeIntake() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.safeIntake();
    // magazineSubsystem.toIntaking();
    magazineSubsystem.setEmpty();
    ledSubsystem.setFlaming();
    setState(RobotStates.TO_INTAKING);
  }

  public void toIntake() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.intake();
    // magazineSubsystem.toIntaking();
    magazineSubsystem.setEmpty();
    ledSubsystem.setFlaming();
    setState(RobotStates.TO_INTAKING);
  }

  public void toAmp() {
    driveSubsystem.setIsAligningShot(false);
    magazineSubsystem.setSpeed(0.0);
    superStructure.amp();
    intakeSubsystem.toEjecting();
    // intakeSubsystem.setPercent(0.0);
    ledSubsystem.setOff();

    setState(RobotStates.TO_AMP);
  }

  public void startShootKnownPos(Pose2d pos) {
    shootPos = pos;
    shootKnownPos = true;
    usingDistance = false;

    driveSubsystem.setIsAligningShot(false);

    double[] shootSolution =
        getShootSolution(driveSubsystem.getDistanceToSpeaker(pos), shootingLookupTable);

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SHOOT);
  }

  // public void startFeed() {
  //   usingDistance = false;
  //   shootKnownPos = false;
  //   driveSubsystem.setIsAligningShot(true);
  //   driveSubsystem.setIsFeeding(true);

  //   double[] feedSolution =
  //       getShootSolution(driveSubsystem.getDistanceToFeedTarget(), feedingLookupTable);

  //   magazineSubsystem.setSpeed(0.0);
  //   superStructure.shoot(feedSolution[0], feedSolution[1], feedSolution[2]);

  //   setState(RobotStates.TO_FEED);
  // }

  public void spinUpShotSolution(Pose2d pose) {
    shootPos = pose;
    double[] shootSolution =
        getShootSolution(driveSubsystem.getDistanceToSpeaker(pose), shootingLookupTable);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    setState(RobotStates.SPIN_UP);
  }

  public void startShoot() {
    usingDistance = false;
    shootKnownPos = false;
    driveSubsystem.setIsAligningShot(true);

    double[] shootSolution =
        getShootSolution(driveSubsystem.getDistanceToSpeaker(), shootingLookupTable);

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SHOOT);
  }

  public void startShootDistance(double distance) {
    usingDistance = true;
    shootKnownPos = false;

    double[] shootSolution = getShootSolution(distance, shootingLookupTable);

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

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
    // magazineSubsystem.setSpeed(0.0);
    superStructure.stow();
    if (ledSubsystem.getState() == LedState.FLAMING) {
      ledSubsystem.setOff();
    }

    setState(RobotStates.TO_STOW);
  }

  public void toDefenseStow() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    magazineSubsystem.setSpeed(0.0);
    superStructure.defenceStow();

    desiredState = RobotStates.INTAKING;

    setState(RobotStates.TO_STOW);
  }

  public void toDefense() {
    driveSubsystem.setIsAligningShot(false);
    superStructure.defense();

    setState(RobotStates.TO_DEFENSE);
  }

  public void toPreparePodium() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    superStructure.preparePodium();
    ledSubsystem.setOff();

    setState(RobotStates.TO_PODIUM);
  }

  public void toSubwoofer() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    superStructure.subwoofer();
    intakeSubsystem.setPercent(0.0);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SUBWOOFER);
  }

  public void toFixedFeeding() {
    ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    superStructure.fixedFeeding(FastMath.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    intakeSubsystem.setPercent(0.0);

    setState(RobotStates.TO_FEED);
  }

  public void prepareClimb() {
    magazineSubsystem.toPrepClimb();
    climbSubsystem.zero(true);
    climbSubsystem.extendForks();
    superStructure.toPrepClimb();

    setState(RobotStates.PREPPING_CLIMB);
  }

  public void climb(boolean continueToTrap, boolean decendAfterTrap) {
    climbSubsystem.trapClimb();
    this.continueToTrap = continueToTrap;
    this.decendClimbAfterTrap = decendAfterTrap;

    setState(RobotStates.CLIMBING);
  }

  public void toTrap() {
    // climbSubsystem.extendTrapBar();
    superStructure.toTrap();

    setState(RobotStates.TO_TRAP);
  }

  public void scoreTrap() {
    scoreTrapTimer.reset();
    scoreTrapTimer.start();
    magazineSubsystem.trap();
    ledSubsystem.setOff();
    setState(RobotStates.SCORE_TRAP);
  }

  // Overload of scoreTrap()
  public void scoreTrap(boolean decend) {
    scoreTrapTimer.reset();
    scoreTrapTimer.start();
    magazineSubsystem.trap();
    this.decendClimbAfterTrap = decend;
    ledSubsystem.setOff();
    setState(RobotStates.SCORE_TRAP);
  }

  public void decendClimb() {
    superStructure.toPrepClimb();

    setState(RobotStates.PREPPING_DECEND);
  }

  public void postClimbStow() {
    toStow();
    climbSubsystem.retractForks();
    climbSubsystem.retractTrapBar();
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
      safeStow = false;
      magazineSubsystem.toReleaseGamePiece();
      setState(RobotStates.RELEASE);
    }
    ledSubsystem.setOff();
  }

  public void toTune() {
    setState(RobotStates.TO_TUNE);
    superStructure.shootTune(elbowOffset);
    hasDelayed = false;
  }

  public void setShootDelay(double delay) {
    shootDelay = delay;
  }

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case TO_STOW:
        if (superStructure.isFinished()) {
          if (desiredState == RobotStates.INTAKING) {
            desiredState = RobotStates.STOW;
            toIntake();
          }

          if (magazineSubsystem.hasPiece()) {
            intakeSubsystem.toReversing();
          } else if (!magazineSubsystem.hasPiece()) {
            toIntake();
            break;
          }

          setState(RobotStates.STOW);
        }
        break;

      case STOW:
        if (magazineHasNote()
            && driveSubsystem.getDistanceToSpeaker() < RobotStateConstants.kLookupMaxDistance) {
          superStructure.spinUp();
        } else {
          superStructure.stopShoot();
        }

        break;

      case TO_INTAKING:
        if (superStructure.isFinished()) {
          intakeSubsystem.toIntaking();
          setState(RobotStates.INTAKING);
        }
        break;

      case INTAKING:
        if (intakeSubsystem.getState() == IntakeState.HAS_PIECE) {
          ledSubsystem.setGreen();
        }
        if (magazineSubsystem.hasPiece()) {
          // Magazine stops running upon detecting a game piece
          intakeSubsystem.setPercent(0);
          toStow();
        }
        if (intakeSubsystem.getState() == IntakeState.HAS_PIECE
            && (magazineSubsystem.getState() != MagazineStates.INTAKING)
            && magazineSubsystem.hasPiece() == false) {
          magazineSubsystem.toIntaking();
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
      case SPIN_UP: // Indicator State
        break;
      case TO_FEED:
        if (superStructure.isFinished()) {
          magazineSubsystem.toEmptying();

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();
          hasShootBeamUnbroken = false;

          setState(RobotStates.SHOOTING);
        }
        // double[] feedSolution =
        //     getShootSolution(driveSubsystem.getDistanceToFeedTarget(), feedingLookupTable);
        // superStructure.shoot(feedSolution[0], feedSolution[1], feedSolution[2]);

        // if (driveSubsystem.isDriveStillFeed()
        //     && driveSubsystem.isPointingAtFeedTarget()
        //     && superStructure.isFinished()) {

        //     org.littletonrobotics.junction.Logger.recordOutput(
        //         "ShootingData/shot" + Integer.toString(curShot) + "/Position", shootPos);
        //     org.littletonrobotics.junction.Logger.recordOutput(
        //         "ShootingData/shot" + Integer.toString(curShot) + "/Distance",
        // grabbedShotDistance);

        //   magazineSubsystem.toEmptying();

        //   curShot += 1;
        //   hasShootBeamUnbroken = false;

        //   setState(RobotStates.SHOOTING);
        // }
        break;

      case TO_SHOOT:
        if (!usingDistance && !shootKnownPos) {
          double[] shootSolution =
              getShootSolution(driveSubsystem.getDistanceToSpeaker(), shootingLookupTable);
          superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
        }

        if (shootKnownPos) {
          double[] shootSolution =
              getShootSolution(driveSubsystem.getDistanceToSpeaker(shootPos), shootingLookupTable);
          superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
          double vomega = driveSubsystem.getvOmegaToGoal(shootPos);
          driveSubsystem.move(0, 0, vomega, true);
        }

        if (isAuto && !usingDistance) {
          double vomega = driveSubsystem.getvOmegaToGoal();
          driveSubsystem.move(0, 0, vomega, true);
        }

        if (driveSubsystem.isDriveStill()
            && (usingDistance ? true : driveSubsystem.isPointingAtGoal())
            && superStructure.isFinished()) {

          if (!shootKnownPos) {
            org.littletonrobotics.junction.Logger.recordOutput(
                "ShootingData/shot" + Integer.toString(curShot) + "/Position",
                driveSubsystem.getPoseMeters());
            org.littletonrobotics.junction.Logger.recordOutput(
                "ShootingData/shot" + Integer.toString(curShot) + "/Distance", grabbedShotDistance);
          } else {
            org.littletonrobotics.junction.Logger.recordOutput(
                "ShootingData/shot" + Integer.toString(curShot) + "/Position", shootPos);
            org.littletonrobotics.junction.Logger.recordOutput(
                "ShootingData/shot" + Integer.toString(curShot) + "/Distance", grabbedShotDistance);
          }
          magazineSubsystem.toEmptying();

          curShot += 1;
          hasShootBeamUnbroken = false;
          if (isAuto) {
            ledSubsystem.setBlue();
          }

          setState(RobotStates.SHOOTING);
        }
        break;

      case SHOOTING:
        if (!hasShootBeamUnbroken && magazineSubsystem.isRevBeamOpen()) {
          logger.info("Note out of Magazine");
          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();
          hasShootBeamUnbroken = true;
        }
        if (hasShootBeamUnbroken) {
          shootDelayTimer.stop();
          driveSubsystem.setIsAligningShot(false);
          driveSubsystem.setIsFeeding(false);
          magazineSubsystem.setSpeed(0);

          superStructure.stopShoot();
          ledSubsystem.setOff();
          toIntake();
          magazineSubsystem.setEmpty();
        }

        break;

      case TO_PODIUM:
        if (magazineSubsystem.getState() != MagazineStates.PREP_PODIUM
            && magazineSubsystem.getState() != MagazineStates.SPEEDUP
            && magazineSubsystem.getState() != MagazineStates.SHOOT
            && superStructure.isShooterAtSpeed()) {
          magazineSubsystem.preparePodium();
        }
        if (magazineSubsystem.getState() == MagazineStates.SPEEDUP) {
          // superStructure.slowWheelSpin();
          superStructure.stopShoot();
        }
        if (magazineSubsystem.getSpeed() >= MagazineConstants.kPodiumRumbleSpeed) {
          ledSubsystem.setBlue();
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
        if (magazineShootDelayTimer.hasElapsed(ShooterConstants.kPodiumShootTime)) {
          magazineShootDelayTimer.stop();

          superStructure.stopPodiumShoot();
          ledSubsystem.setOff();
          toStow();
        }
        break;

      case TO_SUBWOOFER:
        if (superStructure.isFinished()) {
          magazineSubsystem.toEmptying();

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();
          hasShootBeamUnbroken = false;

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
        if (superStructure.isFinished() && !hasDelayed) {
          startShootDelay.reset();
          startShootDelay.start();
          hasDelayed = true;
        }
        if (startShootDelay.hasElapsed(shootDelay) && hasDelayed) {
          magazineSubsystem.toEmptying(magazineTuneSpeed);

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();
          hasShootBeamUnbroken = false;

          setState(RobotStates.SHOOTING);
          hasDelayed = false;
        }
        break;
      case PREPPING_CLIMB:
        if (climbSubsystem.isFinished()
            && climbSubsystem.isForkFinished()
            && climbSubsystem.hasClimbZeroed()
            && superStructure.isFinished()) {
          setState(RobotStates.CLIMB_PREPPED);
        }
        break;
      case CLIMB_PREPPED:
        break;

      case CLIMBING:
        if (climbSubsystem.getPosition() <= RobotStateConstants.kClimbMoveElbowPos) {
          superStructure.toFold();
          setState(RobotStates.FOLDING_OUT);
        }
        // if (climbSubsystem.isFinished()) {
        // setState(RobotStates.FOLDING_OUT);
        // }
        break;

      case FOLDING_OUT:
        if (superStructure.isFinished() && climbSubsystem.isFinished()) {
          if (continueToTrap) toTrap();
          else setState(RobotStates.CLIMBED);
        }
        break;

      case CLIMBED:
        break;

      case TO_TRAP:
        logger.info(
            "Wrist Pos: {}, greater than: {}",
            superStructure.getWristPos(),
            superStructure.getWristPos() >= RobotStateConstants.kMaxWristToMoveTrapBar);
        if (superStructure.getWristPos() >= RobotStateConstants.kMinWristToMoveTrapBar) {
          climbSubsystem.extendTrapBar();
        }
        if (superStructure.isFinished()) {
          // climbSubsystem.extendTrapBar();
          climbTrapTimer.reset();
          climbTrapTimer.start();
          setState(RobotStates.TRAP);
        }
        break;
      case TRAP:
        if (climbTrapTimer.hasElapsed(RobotStateConstants.kClimbTrapTimer)) {
          scoreTrap(); // Transitions to SCORE_TRAP
        }
        // if (!magazineSubsystem.hasPiece()) {
        // climbSubsystem.retractTrapBar();
        // superStructure.toFold();
        // setState(RobotStates.FOLDING_IN);
        // }
        break;
      case SCORE_TRAP:
        if (scoreTrapTimer.hasElapsed(RobotStateConstants.kTrapTimer)) {
          superStructure.toFold();
          setState(RobotStates.FOLDING_IN);
        }
        break;
      case FOLDING_IN:
        if (superStructure.getWristPos() <= RobotStateConstants.kMaxWristToMoveTrapBar) {
          climbSubsystem.retractTrapBar();
        }
        if (superStructure.isFinished()) {
          magazineSubsystem.setSpeed(0.0);
          magazineSubsystem.setEmpty();
          superStructure.toPrepClimb();

          if (decendClimbAfterTrap) {
            setState(RobotStates.PREPPING_DECEND);
          } else {
            setState(RobotStates.CLIMBED);
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
      case TO_DEFENSE:
        if (superStructure.isFinished()
            && superStructure.getState() == SuperStructureStates.DEFENSE) {
          setState(RobotStates.DEFENSE);
        }
        break;
      case DEFENSE:
        break;
      default:
        break;
    }

    org.littletonrobotics.junction.Logger.recordOutput("Robot State", curState);
  }

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("state", () -> curState.ordinal()),
        new Measure("using distance", () -> usingDistance ? 1.0 : 0.0));
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
    SPIN_UP,
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
    SCORE_TRAP,
    FOLDING_OUT,
    FOLDING_IN,
    DESCENDING,
    POST_CLIMB,
    PREPPING_DECEND,
    CLIMBING,
    CLIMBED,
    TO_DEFENSE,
    DEFENSE,
    TO_FEED
  }
}
