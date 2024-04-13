package frc.robot.subsystems.robotState;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.opencsv.CSVReader;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.MagazineConstants;
import frc.robot.constants.RobotConstants;
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
  private AnalogInput breakerTemp;

  private RobotStates curState = RobotStates.IDLE;
  private RobotStates nextState = RobotStates.IDLE;

  private double[][] shootingLookupTable;
  private double[][] feedingLookupTable;

  private Timer shootDelayTimer = new Timer();
  private Timer magazineShootDelayTimer = new Timer();
  private Timer ampStowTimer = new Timer();
  private Timer startShootDelay = new Timer();
  private Timer climbTrapTimer = new Timer();
  private Timer scoreTrapTimer = new Timer();
  private Timer ejectPiecesTimer = new Timer();
  private boolean hasDelayed = false;
  private double shootDelay = 0.0;
  private boolean hasShootBeamUnbroken = false;

  private double[] shootSolution = new double[4];

  private Alliance allianceColor = Alliance.Blue;

  private boolean safeStow = false;
  private boolean decendClimbAfterTrap = false;
  private boolean continueToTrap = false;
  private boolean usingDistance = false;
  private boolean isAuto = false;
  private boolean shootKnownPos = false;
  private boolean inWaitForUnbreakMode = false;
  private boolean movingShoot = false;
  private boolean inDefense = false;
  private Pose2d shootPos;
  private double grabbedShotDistance = 0.0;
  private double magazineTuneSpeed = 0.0;
  private boolean speedUpPass = false;
  private boolean hasStoppedWheels = true;

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
    this.breakerTemp = new AnalogInput(RobotConstants.kBreakerTempChannel);
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
    return magazineSubsystem.hasPiece()
        || intakeSubsystem.isBeamBroken()
        || intakeSubsystem.hasNote();
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
  private double[][] parseLookupTable(String path) {
    double[][] lookupTable;
    List<String[]> list = new LinkedList<>();
    List<double[]> listD = new LinkedList<>();
    try {
      CSVReader csvReader = new CSVReader(new FileReader(path));
      list = csvReader.readAll();
      int line = 0;
      for (String[] sArr : list) {
        if (line == 0) {
          line++;
          continue;
        }
        double[] dArr = new double[sArr.length];
        for (int i = 0; i < sArr.length; ++i) dArr[i] = Double.parseDouble(sArr[i]);
        listD.add(dArr);
      }

    } catch (Exception e) {
      logger.warn("Failed to read lookup table at {} due to {}", path, e);
    }

    double[][] doubleArr = new double[listD.size()][];
    lookupTable = listD.toArray(doubleArr);

    return lookupTable;
  }

  private void getShootSolution(double distance, double[][] table) {
    // logger.info(
    //     "Timestamp Before Starting Parse: {}",
    //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
    int index;
    distance += RobotStateConstants.kDistanceOffset;
    grabbedShotDistance = distance;
    if (distance < RobotStateConstants.kLookupMinDistance) {
      index = 0;
      logger.warn(
          "Distance {} is less than min distance in table {}",
          distance,
          RobotStateConstants.kLookupMinDistance);
    } else if (distance > RobotStateConstants.kLookupMaxDistance) {
      index = shootingLookupTable.length - 1;
      logger.warn(
          "Distance {} is more than max distance in table {}",
          distance,
          RobotStateConstants.kLookupMaxDistance);
    } else {
      index =
          (int)
              ((distance - RobotStateConstants.kLookupMinDistance)
                      / RobotStateConstants.kDistanceIncrement
                  + 0.0);
      //   logger.info("Distance: {} | Measured {}", shootingLookupTable[index][0], distance);
      /*
       * index =
       * (int) (Math.round(distance) - RobotStateConstants.kLookupMinDistance)
       * / RobotStateConstants.kDistanceIncrement;
       */
    }

    // logger.info("Left Shooter: {}", shootingLookupTable[index][1]);

    // logger.info(
    //     "Timestamp Before Parsing Doubles: {}",
    //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
    shootSolution[0] = table[index][1]; // Left Shooter
    shootSolution[1] = table[index][2]; // Right Shooter
    shootSolution[2] = table[index][3] + elbowOffset; // Elbow
    shootSolution[3] = table[index][4];
    // logger.info(
    //     "Timestamp AFter Parsing Doubles: {}",
    //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
  }

  public boolean getIsAuto() {
    return isAuto;
  }

  public boolean getIsInDefense() {
    return inDefense;
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
    magazineSubsystem.toAmp();
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

    getShootSolution(driveSubsystem.getDistanceToSpeaker(pos), shootingLookupTable);

    intakeSubsystem.toEjecting();
    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SHOOT);
  }

  public void startFeed() {
    usingDistance = false;
    shootKnownPos = false;
    driveSubsystem.setIsAligningShot(true);
    driveSubsystem.setIsFeeding(true);

    getShootSolution(driveSubsystem.getDistanceToFeedTarget(), feedingLookupTable);

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.TO_FEED);
  }

  public void spinUpShotSolution(Pose2d pose) {
    shootPos = pose;
    getShootSolution(driveSubsystem.getDistanceToSpeaker(pose), shootingLookupTable);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    setState(RobotStates.SPIN_UP);
  }

  public void startShoot() {
    usingDistance = false;
    shootKnownPos = false;
    movingShoot = false;

    intakeSubsystem.toEjecting();
    driveSubsystem.setIsAligningShot(true);

    getShootSolution(driveSubsystem.getDistanceToSpeaker(), shootingLookupTable);

    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SHOOT);
  }

  public void startShootDistance(double distance) {
    usingDistance = true;
    shootKnownPos = false;
    movingShoot = false;

    getShootSolution(distance, shootingLookupTable);

    intakeSubsystem.toEjecting();
    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
    ledSubsystem.setOff();

    setState(RobotStates.TO_SHOOT);
  }

  public void startMovingShoot() {
    usingDistance = false;
    shootKnownPos = false;
    movingShoot = true;

    driveSubsystem.setIsAligningShot(true);
    driveSubsystem.setIsMoveAndShoot(true);

    intakeSubsystem.toEjecting();
    Translation2d virtualT = driveSubsystem.getPoseMeters().getTranslation();
    ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
    getShootSolution(
        driveSubsystem.getDistanceToSpeaker(
            new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
        shootingLookupTable);

    for (int i = 0; i < RobotStateConstants.kMoveWhileShootIterations; i++) {
      virtualT =
          virtualT.plus(
              new Translation2d(
                  speeds.vxMetersPerSecond * shootSolution[3],
                  speeds.vyMetersPerSecond * shootSolution[3]));
      getShootSolution(
          driveSubsystem.getDistanceToSpeaker(
              new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
          shootingLookupTable);
    }
    driveSubsystem.setMoveAndShootVirtualPose(
        new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation()));
    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.TO_MOVING_SHOOT);
  }

  public void startMovingFeed() {
    usingDistance = false;
    shootKnownPos = false;
    movingShoot = true;

    driveSubsystem.setIsAligningShot(true);
    driveSubsystem.setIsMoveAndShoot(true);

    intakeSubsystem.toEjecting();
    Translation2d virtualT = driveSubsystem.getPoseMeters().getTranslation();
    ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
    getShootSolution(
        driveSubsystem.getDistanceToSpeaker(
            new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
        feedingLookupTable);

    for (int i = 0; i < RobotStateConstants.kMoveWhileShootIterations; i++) {
      virtualT =
          virtualT.plus(
              new Translation2d(
                  speeds.vxMetersPerSecond * shootSolution[3],
                  speeds.vyMetersPerSecond * shootSolution[3]));
      getShootSolution(
          driveSubsystem.getDistanceToSpeaker(
              new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
          feedingLookupTable);
    }
    driveSubsystem.setMoveAndShootVirtualPose(
        new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation()));
    magazineSubsystem.setSpeed(0.0);
    superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

    setState(RobotStates.TO_MOVING_FEED);
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
    climbSubsystem.punchAir();
    ledSubsystem.setCandy();
    inDefense = true;
    // setState(RobotStates.DEFENSE);
  }

  public void toPreparePodium() {
    driveSubsystem.setIsAligningShot(false);
    intakeSubsystem.setPercent(0.0);
    superStructure.preparePodium();
    ledSubsystem.setOff();

    setState(RobotStates.TO_PODIUM);
  }

  public void toSubwoofer() {
    if (speedUpPass) {
      // Low Feed shot
      driveSubsystem.setIsAligningShot(false);
      intakeSubsystem.setPercent(0.0);
      superStructure.lowFeedShot();
      intakeSubsystem.setPercent(0.0);
      ledSubsystem.setOff();

      setState(RobotStates.TO_FEED);
    } else {
      // Subwoofer
      driveSubsystem.setIsAligningShot(false);
      intakeSubsystem.setPercent(0.0);
      superStructure.subwoofer();
      intakeSubsystem.setPercent(0.0);
      ledSubsystem.setOff();

      setState(RobotStates.TO_SUBWOOFER);
    }
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
    inDefense = false;
    magazineSubsystem.toPrepClimb();
    climbSubsystem.zero(true);
    climbSubsystem.extendForks();
    superStructure.toPrepClimb();

    setState(RobotStates.PREPPING_CLIMB);
  }

  public void climb(boolean continueToTrap, boolean decendAfterTrap) {
    inDefense = false;
    climbSubsystem.trapClimb();
    this.continueToTrap = continueToTrap;
    this.decendClimbAfterTrap = decendAfterTrap;

    setState(RobotStates.CLIMBING);
  }

  public void toTrap() {
    inDefense = false;
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
    inDefense = false;
    toStow();
    climbSubsystem.retractForks();
    climbSubsystem.retractTrapBar();
    climbSubsystem.stow();

    setState(RobotStates.TO_STOW);
  }

  // FIXME
  public void releaseGamePiece() {
    if (curState == RobotStates.TO_PODIUM || curState == RobotStates.TO_PODIUM) {
      superStructure.podiumShoot();

      magazineShootDelayTimer.stop();
      magazineShootDelayTimer.reset();
      magazineShootDelayTimer.start();

      setState(RobotStates.PODIUM_SHOOTING);
    } else if (curState == RobotStates.AMP || curState == RobotStates.TO_AMP) {
      safeStow = false;
      magazineSubsystem.toReleaseGamePiece();
      setState(RobotStates.RELEASE);
    } else {
      toFixedFeeding();
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

  public void togglePassSpeedUp() {
    speedUpPass = !speedUpPass;
  }

  public boolean isPassSpeedUp() {
    return speedUpPass;
  }

  public void toEjecting() {
    ejectPiecesTimer.stop();
    ejectPiecesTimer.reset();
    ejectPiecesTimer.start();
    // intakeSubsystem.toEjecting();
    magazineSubsystem.toEjecting();
    superStructure.ejecting();
    setState(RobotStates.EJECTING);
  }

  // Periodic
  @Override
  public void periodic() {
    if (!isAuto && DriverStation.getMatchTime() <= 25.0 && DriverStation.isTeleopEnabled()) {
      ledSubsystem.setBlinking(true);
    } else {
      ledSubsystem.setBlinking(false);
    }

    if (speedUpPass) {
      hasStoppedWheels = false;
      ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
      superStructure.fixedFeeding(
          FastMath.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    } else {
      //   superStructure.stopShoot();
      if (!hasStoppedWheels) superStructure.stopShoot();
      hasStoppedWheels = true;
    }

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
        } else if (speedUpPass) {
          ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
          superStructure.fixedFeeding(
              FastMath.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
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
        if (magazineSubsystem.hasPiece()) {
          // Magazine stops running upon detecting a game piece
          intakeSubsystem.setPercent(0);
          ledSubsystem.setBlue();
          toStow();
          break;
        }
        if (intakeSubsystem.getState() == IntakeState.HAS_PIECE) {
          ledSubsystem.setGreen();

          if (!intakeSubsystem.isBeamBroken() && !magazineSubsystem.hasPiece()) {
            magazineSubsystem.toIntaking(false);
          }
        }
        if (intakeSubsystem.getState() == IntakeState.HAS_PIECE
            && (magazineSubsystem.getState() != MagazineStates.INTAKING)
            && magazineSubsystem.hasPiece() == false) {
          magazineSubsystem.toIntaking(true);
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
          inWaitForUnbreakMode = false;

          setState(RobotStates.SHOOTING);
        }
        // getShootSolution(driveSubsystem.getDistanceToFeedTarget(), feedingLookupTable);
        // superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

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

      case TO_MOVING_FEED:
        // Approximate future position of robot
        Translation2d virtualT = driveSubsystem.getPoseMeters().getTranslation();
        ChassisSpeeds speeds = driveSubsystem.getFieldRelSpeed();
        getShootSolution(
            driveSubsystem.getDistanceToSpeaker(
                new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
            feedingLookupTable);

        for (int i = 0; i < RobotStateConstants.kMoveWhileShootIterations; i++) {
          virtualT =
              virtualT.plus(
                  new Translation2d(
                      speeds.vxMetersPerSecond * shootSolution[3],
                      speeds.vyMetersPerSecond * shootSolution[3]));
          getShootSolution(
              driveSubsystem.getDistanceToSpeaker(
                  new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
              feedingLookupTable);
        }

        superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

        Pose2d virtualPos = new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation());
        driveSubsystem.setMoveAndShootVirtualPose(virtualPos);

        if (driveSubsystem.isPointingAtGoal(virtualPos)
            && superStructure.isFinished()
            && driveSubsystem.isDriveStillFeed()) {

          org.littletonrobotics.junction.Logger.recordOutput(
              "ShootingData/shot" + Integer.toString(curShot) + "/Position", virtualPos);
          org.littletonrobotics.junction.Logger.recordOutput(
              "ShootingData/shot" + Integer.toString(curShot) + "/Distance", grabbedShotDistance);

          magazineSubsystem.toEmptying();

          curShot += 1;
          hasShootBeamUnbroken = false;

          inWaitForUnbreakMode = false;
          hasShootBeamUnbroken = false;
          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }
        break;

      case TO_SHOOT:
        if (!usingDistance && !shootKnownPos) {
          //   logger.info(
          //       "Timestamp Before Shot Sol: {}",
          //       org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
          getShootSolution(driveSubsystem.getDistanceToSpeaker(), shootingLookupTable);
          //   logger.info(
          //       "Timestamp After Shot Sol Before Shoot: {}",
          //       org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
          superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
          //   logger.info(
          //       "Timestamp After Shoot: {}",
          //       org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
        }

        if (shootKnownPos) {
          getShootSolution(driveSubsystem.getDistanceToSpeaker(shootPos), shootingLookupTable);
          superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);
          double vomega = driveSubsystem.getvOmegaToGoal(shootPos);
          driveSubsystem.move(0, 0, vomega, true);
        }

        if (isAuto && !usingDistance) {

          double vomega = driveSubsystem.getvOmegaToGoal();
          driveSubsystem.move(0, 0, vomega, true);
        }
        // logger.info(
        //     "Timestamp Before Conditions: {}",
        //     org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);
        if (driveSubsystem.isDriveStill()
            && (usingDistance ? true : driveSubsystem.isPointingAtGoal())
            && superStructure.isFinished()) {
          //   logger.info(
          //       "Timestamp After Conditions: {}",
          //       org.littletonrobotics.junction.Logger.getRealTimestamp() / 1000);

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
          inWaitForUnbreakMode = false;
          if (isAuto) {
            ledSubsystem.setBlue();
          }

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();
          hasShootBeamUnbroken = false;
          inWaitForUnbreakMode = false;

          setState(RobotStates.SHOOTING);
        }
        break;

      case TO_MOVING_SHOOT:
        // Approximate future position of robot
        virtualT = driveSubsystem.getPoseMeters().getTranslation();
        speeds = driveSubsystem.getFieldRelSpeed();
        getShootSolution(
            driveSubsystem.getDistanceToSpeaker(
                new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
            shootingLookupTable);

        for (int i = 0; i < RobotStateConstants.kMoveWhileShootIterations; i++) {
          virtualT =
              virtualT.plus(
                  new Translation2d(
                      speeds.vxMetersPerSecond * shootSolution[3],
                      speeds.vyMetersPerSecond * shootSolution[3]));
          getShootSolution(
              driveSubsystem.getDistanceToSpeaker(
                  new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation())),
              shootingLookupTable);
        }

        superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

        virtualPos = new Pose2d(virtualT, driveSubsystem.getPoseMeters().getRotation());
        driveSubsystem.setMoveAndShootVirtualPose(virtualPos);

        if (driveSubsystem.isPointingAtGoal(virtualPos)
            && superStructure.isFinished()
            && driveSubsystem.isMoveShootAllowed(isAuto)) {

          org.littletonrobotics.junction.Logger.recordOutput(
              "ShootingData/shot" + Integer.toString(curShot) + "/Position", virtualPos);
          org.littletonrobotics.junction.Logger.recordOutput(
              "ShootingData/shot" + Integer.toString(curShot) + "/Distance", grabbedShotDistance);

          magazineSubsystem.toEmptying();

          curShot += 1;
          hasShootBeamUnbroken = false;
          if (isAuto) {
            ledSubsystem.setBlue();
          }

          inWaitForUnbreakMode = false;
          hasShootBeamUnbroken = false;
          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }
        break;

      case SHOOTING:
        if (!hasShootBeamUnbroken && magazineSubsystem.isRevBeamOpen()) {
          logger.info("Note out of Magazine");
          // shootDelayTimer.stop();
          // shootDelayTimer.reset();
          // shootDelayTimer.start();
          hasShootBeamUnbroken = true;
        }
        if (isAuto
            || (shootDelayTimer.hasElapsed(RobotStateConstants.kShootDelay)
                && !inWaitForUnbreakMode)) {
          inWaitForUnbreakMode = true;
        }
        if (hasShootBeamUnbroken && inWaitForUnbreakMode) {
          shootDelayTimer.stop();
          driveSubsystem.setIsAligningShot(false);
          driveSubsystem.setIsFeeding(false);
          driveSubsystem.setIsMoveAndShoot(false);
          magazineSubsystem.setSpeed(0);

          superStructure.stopShoot();
          ledSubsystem.setOff();
          toIntake();
          magazineSubsystem.setEmpty();
          inWaitForUnbreakMode = false;
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
          inWaitForUnbreakMode = false;

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
          inWaitForUnbreakMode = false;

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
          climbSubsystem.trapClimbAdjust();
          setState(RobotStates.ADJUST_TRAP);
        }
        break;
      case ADJUST_TRAP:
        if (climbSubsystem.isFinished()) {
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
      case DEFENSE:
        break;
      case EJECTING:
        if (ejectPiecesTimer.hasElapsed(RobotStateConstants.kEjectTimer)) {
          toIntake();
        }
        break;
      default:
        break;
    }

    org.littletonrobotics.junction.Logger.recordOutput("States/Robot State", curState);
    org.littletonrobotics.junction.Logger.recordOutput("BreakerTemp", breakerTemp.getValue());
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
    TO_MOVING_SHOOT,
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
    ADJUST_TRAP,
    FOLDING_OUT,
    FOLDING_IN,
    DESCENDING,
    POST_CLIMB,
    PREPPING_DECEND,
    CLIMBING,
    CLIMBED,
    DEFENSE,
    TO_FEED,
    EJECTING,
    TO_MOVING_FEED
  }
}
