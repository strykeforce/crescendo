package frc.robot.subsystems.superStructure;

import frc.robot.constants.SuperStructureConstants;
import frc.robot.subsystems.elbow.ElbowSubsystem;
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
  WristSubsystem wristSubsystem;
  ElbowSubsystem elbowSubsystem;
  ShooterSubsystem shooterSubsystem;
  Logger logger;
  SuperStructureStates curState = SuperStructureStates.IDLE;
  SuperStructureStates nextState = SuperStructureStates.IDLE;
  private double leftShooterSpeed = 0.0;
  private double rightShooterSpeed = 0.0;
  private double elbowSetpoint = 0.0;
  private double wristSetpoint = 0.0;
  private boolean flipMagazineOut = false;

  // Constructor
  public SuperStructure(
      WristSubsystem wristSubsystem,
      ElbowSubsystem elbowSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    logger = LoggerFactory.getLogger(this.getClass());
  }

  // Getter/Setter Methods
  public boolean isFinished() {
    return elbowSubsystem.isFinished() && wristSubsystem.isFinished() && shooterSubsystem.atSpeed();
  }

  public SuperStructureStates getState() {
    return curState;
  }

  // public void setState(SuperStructureStates state) {
  //   if (state != SuperStructureStates.IDLE) {
  //     curState = SuperStructureStates.TRANSFER;
  //     nextState = state;
  //   } else {
  //     curState = state;
  //   }
  // }

  // Helper Methods

  // Basic methods to go to each position
  //    Works by setting each axes' setpoint and starting intial movement
  //      Then determines if elbow or wrist go first by flipMagazineOut boolean
  public void shoot(double leftShooterSpeed, double rightShooterSpeed, double elbowSetpoint) {
    this.elbowSetpoint = elbowSetpoint;
    this.leftShooterSpeed = leftShooterSpeed;
    this.rightShooterSpeed = rightShooterSpeed;
    wristSetpoint = SuperStructureConstants.kWristShootSetPoint;

    shooterSubsystem.setLeftSpeed(leftShooterSpeed);
    shooterSubsystem.setRightSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(SHOOTING)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SHOOTING;
  }

  public void preClimb() {
    elbowSetpoint = SuperStructureConstants.kElbowPreClimbSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPreClimbSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(PRE_CLIMB)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.PRE_CLIMB;
  }

  public void postClimb() {
    elbowSetpoint = SuperStructureConstants.kElbowPostClimbSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPostClimbSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;

    shooterSubsystem.setSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(POST_CLIMB)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.POST_CLIMB;
  }

  public void amp() {
    elbowSetpoint = SuperStructureConstants.kElbowAmpSetPoint;
    wristSetpoint = SuperStructureConstants.kWristAmpSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterAmpSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterAmpSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(AMP)");
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.AMP;
  }

  public void trap() {
    elbowSetpoint = SuperStructureConstants.kElbowTrapSetPoint;
    wristSetpoint = SuperStructureConstants.kWristTrapSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterTrapSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterTrapSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(TRAP)");
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.TRAP;
  }

  public void intake() {
    elbowSetpoint = SuperStructureConstants.kElbowIntakeSetPoint;
    wristSetpoint = SuperStructureConstants.kWristIntakeSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;

    shooterSubsystem.setSpeed(rightShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(INTAKE)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.INTAKE;
  }

  public void defense() {
    elbowSetpoint = SuperStructureConstants.kElbowDefenseSetPoint;
    wristSetpoint = SuperStructureConstants.kWristDefenseSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterDefenseSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterDefenseSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(DEFENSE)");
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.DEFENSE;
  }

  public void stow() {
    elbowSetpoint = SuperStructureConstants.kElbowStowSetPoint;
    wristSetpoint = SuperStructureConstants.kWristStowSetPoint;
    rightShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;
    leftShooterSpeed = SuperStructureConstants.kShooterStowSetPoint;

    shooterSubsystem.setSpeed(leftShooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(DEFENSE)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.DEFENSE;
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
          if (elbowSubsystem.getPosition() > SuperStructureConstants.kElbowMinToMoveWrist) {
            wristSubsystem.setPosition(wristSetpoint);
          }
        } else {
          if (wristSubsystem.getPosition() > SuperStructureConstants.kWristMinToMoveElbow) {
            elbowSubsystem.setPosition(elbowSetpoint);
          }
        }

        // Once all subsystems are at position go into the desired state
        if (isFinished()) {
          logger.info("TRANSFER -> {}", nextState);
          curState = nextState;
        }
        break;

      case SHOOTING:
        break;
      case AMP:
        break;
      case PRE_CLIMB:
        break;
      case TRAP:
        break;
      case POST_CLIMB:
        break;
      case INTAKE:
        break;
      case DEFENSE:
        break;
      case STOW:
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

  // State Enum
  public enum SuperStructureStates {
    IDLE,
    SHOOTING,
    AMP,
    PRE_CLIMB,
    TRAP,
    POST_CLIMB,
    TRANSFER,
    INTAKE,
    DEFENSE,
    STOW
  }
}
