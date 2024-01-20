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

  public void setState(RobotStates robotState) {
    if (this.curState != robotState) {
      logger.info("{} -> {}", this.curState, robotState);
      this.curState = robotState;
    }
  }

  public Alliance getAllianceColor() {
    return Alliance.Red;
    // FIXME
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

    nextState = RobotStates.SHOOT_ALIGN;
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

      case SHOOT_ALIGN:
        if (driveSubsystem.isVelocityStable() && driveSubsystem.isPointingAtGoal()) {
          double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());

          superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

          setState(RobotStates.SHOOT_AIM);
        }

        break;

      case SHOOT_AIM:
        if (superStructure.isFinished()) {
          driveSubsystem.setIsAligningShot(false);

          magazineSubsystem.setSpeed(MagazineConstants.kFeedingSpeed);

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }
        break;

      case SHOOTING:
        if (shootDelayTimer.hasElapsed(ShooterConstants.kShootTime)) {
          shootDelayTimer.stop();

          magazineSubsystem.setSpeed(0);

          setState(RobotStates.STOW);
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
    AMP,
    STOW,
    SHOOT_ALIGN,
    SHOOTING,
    SHOOT_AIM
  }
}
