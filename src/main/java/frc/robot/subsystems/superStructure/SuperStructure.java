package frc.robot.subsystems.superStructure;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class SuperStructure extends MeasurableSubsystem {
  // Private Variables
  private WristSubsystem wristSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private Logger logger;
  private SuperStructureStates curState = SuperStructureStates.IDLE;
  private SuperStructureStates nextState = SuperStructureStates.IDLE;
  private double leftShooterSpeed = 0.0;
  private double rightShooterSpeed = 0.0;
  private double elbowSetpoint = 0.0;
  private double nextElbowSetpoint = 0.0;
  private double wristSetpoint = 0.0;
  private boolean flipMagazineOut = false;
  private Timer timer = new Timer();

  // Tuning
  private double elbowTunePoint = SuperStructureConstants.kElbowIntakeSetPoint;
  private double leftTunePoint = 0.0;
  private double rightTunePoint = 0.0;
  private boolean isAuto = false;
  private boolean isPrecise = false;

  // Constructor
  public SuperStructure(
      WristSubsystem wristSubsystem,
      ElbowSubsystem elbowSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    logger = LoggerFactory.getLogger(this.getClass());
  }

  // Getter/Setter Methods
  public boolean isFinished() {
    return elbowSubsystem.isFinished()
        && wristSubsystem.isFinished()
        && (shooterSubsystem.atSpeed());
  }

  public boolean isElbowZeroed() {
    return elbowSubsystem.hasZeroed();
  }

  public SuperStructureStates getState() {
    return curState;
  }

  public void setIsAuto(boolean val) {
    isAuto = val;
  }

  public double getWristPos() {
    return wristSubsystem.getPosition();
  }

  // public void setState(SuperStructureStates state) {
  // if (state != SuperStructureStates.IDLE) {
  // curState = SuperStructureStates.TRANSFER;
  // nextState = state;
  // } else {
  // curState = state;
  // }
  // }

  public void spinUpWheels(double lSpeed, double rSpeed) {
    shooterSubsystem.setLeftSpeed(lSpeed);
    shooterSubsystem.setRightSpeed(rSpeed);
  }

  // Helper Methods
  public void zeroElbow() {
    elbowSubsystem.zero();
  }

  public void stopShoot() {
    logger.info("Stop Shooter Wheels");
    if (!isAuto) shooterSubsystem.setSpeed(0.0);
  }

  public void slowWheelSpin() {
    shooterSubsystem.setSpeed(-0.5);
  }

  // FIXME needs a better name
  public void spinUp() {
    shooterSubsystem.setLeftSpeed(SuperStructureConstants.kShooterSpinUpLeftSetPoint);
    shooterSubsystem.setRightSpeed(SuperStructureConstants.kShooterSpinUpRightSetPoint);
  }

  public void stopPodiumShoot() {
    logger.info("Stop Magazine Belts");
    magazineSubsystem.setSpeed(0.0);
  }

  public void saveSetpoint(
      double leftShooterSpeed, double rightShooterSpeed, double elbowSetpoint) {
    leftTunePoint = leftShooterSpeed;
    rightTunePoint = rightShooterSpeed;
    elbowTunePoint = elbowSetpoint;
  }

  public void shootTune() {
    wristSubsystem.setPosition(SuperStructureConstants.kWristIntakeSetPoint);
    elbowSubsystem.setPosition(elbowTunePoint);

    shooterSubsystem.setLeftSpeed(leftTunePoint);
    shooterSubsystem.setRightSpeed(rightTunePoint);

    elbowSetpoint = elbowTunePoint;

    logger.info("{} -> TRANSFER(SHOOTING)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SHOOTING;
  }

  // Basic methods to go to each position
  // Works by setting each axes' setpoint and starting intial movement
  // Then determines if elbow or wrist go first by flipMagazineOut boolean
  public void shoot(double leftShooterSpeed, double rightShooterSpeed, double elbowSetpoint) {
    this.elbowSetpoint = elbowSetpoint;
    this.leftShooterSpeed = leftShooterSpeed;
    this.rightShooterSpeed = rightShooterSpeed;
    wristSetpoint = SuperStructureConstants.kWristShootSetPoint;
    shooterSubsystem.setLeftSpeed(leftShooterSpeed);
    shooterSubsystem.setRightSpeed(rightShooterSpeed);

    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(SHOOTING)", curState);
    isPrecise = true;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SHOOTING;
  }

  public void amp() {
    elbowSetpoint = SuperStructureConstants.kElbowAmpSetPoint;
    wristSetpoint = SuperStructureConstants.kWristAmpSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterAmpSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterAmpSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(AMP)", curState);
    isPrecise = false;
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.AMP;
  }

  public void safeIntake() {
    elbowSetpoint = SuperStructureConstants.kElbowIntakeSetPoint;
    wristSetpoint = SuperStructureConstants.kWristIntakeSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;

    shooterSubsystem.setSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(INTAKE)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.INTAKE;
  }

  public void intake() {
    elbowSetpoint = SuperStructureConstants.kElbowIntakeSetPoint;
    wristSetpoint = SuperStructureConstants.kWristIntakeSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;
    if (!isAuto) {
      shooterSubsystem.setSpeed(rightShooterSpeed);
    }
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(SuperStructureConstants.kElbowMinToMoveWrist);

    logger.info("{} -> TRANSFER(INTAKE)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.INTAKE;
  }

  public void defense() {
    elbowSetpoint = SuperStructureConstants.kElbowDefenceIntermediateSetPoint;
    nextElbowSetpoint = SuperStructureConstants.kElbowDefenseFinalSetPoint;
    wristSetpoint = SuperStructureConstants.kWristDefenseSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterDefenseSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterDefenseSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> SAFE_TRANSFER_ELBOW(DEFENSE)", curState);
    isPrecise = false;
    flipMagazineOut = true;
    curState = SuperStructureStates.SAFE_TRANSFER_ELBOW;
    nextState = SuperStructureStates.DEFENSE;
  }

  public void defenceStow() {
    elbowSetpoint = SuperStructureConstants.kElbowDefenceIntermediateSetPoint;
    nextElbowSetpoint = SuperStructureConstants.kElbowStowSetPoint;
    wristSetpoint = SuperStructureConstants.kWristStowSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> SAFE_TRANSFER_ELBOW(STOW)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.SAFE_TRANSFER_ELBOW;
    nextState = SuperStructureStates.STOW;
  }

  public void safeStow() {
    elbowSetpoint = SuperStructureConstants.kElbowStowSetPoint;
    wristSetpoint = SuperStructureConstants.kWristStowSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(STOW)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.STOW;
  }

  public void stow() {
    elbowSetpoint = SuperStructureConstants.kElbowStowSetPoint;
    wristSetpoint = SuperStructureConstants.kWristStowSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;
    if (!isAuto) {
      shooterSubsystem.setSpeed(leftShooterSpeed);
    }
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(SuperStructureConstants.kElbowMinToMoveWrist);

    logger.info("{} -> TRANSFER(STOW)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.STOW;
  }

  public void subwoofer() {
    elbowSetpoint = SuperStructureConstants.kElbowSubwooferSetPoint;
    wristSetpoint = SuperStructureConstants.kWristSubwooferSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterSubwooferSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterSubwooferSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(SuperStructureConstants.kElbowMinToMoveWrist);

    logger.info("{} -> TRANSFER(SUBWOOFER)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SUBWOOFER;
  }

  public void fixedFeeding() {
    elbowSetpoint = SuperStructureConstants.kElbowSubwooferSetPoint;
    wristSetpoint = SuperStructureConstants.kWristSubwooferSetPoint;
    leftShooterSpeed = SuperStructureConstants.kLeftShooterFeedingSetPoint;
    rightShooterSpeed = SuperStructureConstants.kRightShooterFeedingSetPoint;

    shooterSubsystem.setLeftSpeed(leftShooterSpeed);
    shooterSubsystem.setRightSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(SuperStructureConstants.kElbowMinToMoveWrist);

    logger.info("{} -> TRANSFER(FEEDING)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SUBWOOFER;
  }

  public void preparePodium() {
    elbowSetpoint = SuperStructureConstants.kElbowPodiumPrepSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPodiumPrepSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterPodiumPrepSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterPodiumPrepSetPoint;

    shooterSubsystem.setPercent(leftShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(PREP_PODIUM)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.PREP_PODIUM;
  }

  public void toPrepClimb() {
    elbowSetpoint = SuperStructureConstants.kElbowPreClimbSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPreClimbSetPoint;
    leftShooterSpeed = rightShooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;

    shooterSubsystem.setSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(PREP_CLIMB)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.PREP_CLIMB;
  }

  public void toFold() {
    elbowSetpoint = SuperStructureConstants.kElbowFoldedSetPoint;
    wristSetpoint = SuperStructureConstants.kWristFoldedSetPoint;

    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(FOLDING)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.FOLDED;
  }

  public void toTrap() {
    elbowSetpoint = SuperStructureConstants.kElbowTrapSetPoint;
    wristSetpoint = SuperStructureConstants.kWristTrapSetPoint;

    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(TRAP)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.TRAP;
  }

  public void podiumShoot() {
    elbowSetpoint = SuperStructureConstants.kElbowPodiumSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPodiumSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterPodiumSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterPodiumSetPoint;

    shooterSubsystem.setSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(PODIUM)", curState);
    isPrecise = false;
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.PODIUM;
  }

  // Periodic
  @Override
  public void periodic() {
    switch (curState) {
      case IDLE:
        break;
      case TRANSFER:
        // Logic to determine how to move axis based on what the final position is
        if (flipMagazineOut) {
          if (elbowSubsystem.getPosition() < SuperStructureConstants.kElbowMinToMoveWrist) {
            wristSubsystem.setPosition(wristSetpoint);
          }
        } else {
          // FIXME
          if (wristSubsystem.getPosition() < SuperStructureConstants.kWristMinToMoveElbow) {
            elbowSubsystem.setPosition(elbowSetpoint, isPrecise);
          }
        }
        // Once all subsystems are at position go into the desired state
        if (isFinished()) {
          logger.info("TRANSFER -> {}", nextState);
          curState = nextState;
        }
        break;
      case SAFE_TRANSFER_ELBOW:
        if (isFinished()) {
          elbowSetpoint = nextElbowSetpoint;
          wristSubsystem.setPosition(wristSetpoint);
          curState = SuperStructureStates.SAFE_TRANSFER_WRIST;
        }

        break;

      case SAFE_TRANSFER_WRIST:
        if (isFinished()) {
          flipMagazineOut = false;
          curState = SuperStructureStates.TRANSFER;
        }
        break;

      case SHOOTING:
        break;
      case AMP:
        break;
      case INTAKE:
        break;
      case DEFENSE:
        break;
      case STOW:
        break;
      case PREP_PODIUM:
        break;
      case PODIUM:
        break;
      case SUBWOOFER:
        break;
      case FEEDING:
        break;
      case PREP_CLIMB:
        break;
      case FOLDED:
        break;
      case TRAP:
        break;
      case POST_CLIMB:
        break;
    }
    org.littletonrobotics.junction.Logger.recordOutput("SuperStructState", curState);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Elbow Finished", elbowSubsystem.isFinished());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Wrist Finished", wristSubsystem.isFinished());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter Finished", shooterSubsystem.atSpeed());
    org.littletonrobotics.junction.Logger.recordOutput("SuperStructure Is Precise", isPrecise);
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

  // State Enum
  public enum SuperStructureStates {
    IDLE,
    SHOOTING,
    AMP,
    PREP_CLIMB,
    TRAP,
    FOLDED,
    POST_CLIMB,
    TRANSFER,
    INTAKE,
    DEFENSE,
    STOW,
    PODIUM,
    PREP_PODIUM,
    SUBWOOFER,
    SAFE_TRANSFER_ELBOW,
    SAFE_TRANSFER_WRIST,
    FEEDING
  }
}
