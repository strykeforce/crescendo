package frc.robot.subsystems.robotState;

import com.opencsv.CSVReader;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
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

  public Alliance getAllianceColor() {
    return Alliance.Red;
    // FIXME
  }

  // Private Variables
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

  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);

  // Constructor
  public RobotStateSubsystem(
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
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

  // Helper Methods
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
      index = (int) (Math.round(distance) - RobotStateConstants.kLookupMinDistance);
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

      case SHOOT_ALIGN:
        double[] shootSolution = getShootSolution(driveSubsystem.getDistanceToSpeaker());

        superStructure.shoot(shootSolution[0], shootSolution[1], shootSolution[2]);

        if (driveSubsystem.isVelocityStable() && superStructure.isFinished()) {
          driveSubsystem.setIsAligningShot(false);

          magazineSubsystem.setSpeed(0.0 /* kFeedingSpeed */);

          shootDelayTimer.stop();
          shootDelayTimer.reset();
          shootDelayTimer.start();

          setState(RobotStates.SHOOTING);
        }

        break;

      case SHOOTING:
        if (shootDelayTimer.hasElapsed(0.0 /* kShootTime */)) {
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
    STOW,
    SHOOT_ALIGN,
    SHOOTING
  }
}
