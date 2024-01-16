package frc.robot.subsystems.superStructure;

import frc.robot.constants.SuperStructureConstants;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
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
  private double shooterSpeed = 0.0;
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
  public void shoot(double shooterSpeed, double elbowSetpoint) {
    this.elbowSetpoint = elbowSetpoint;
    this.shooterSpeed = shooterSpeed;
    wristSetpoint = SuperStructureConstants.kWristShootSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(SHOOTING)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.SHOOTING;
  }

  public void preClimb() {
    elbowSetpoint = SuperStructureConstants.kElbowPreClimbSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPreClimbSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(PRE_CLIMB)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.PRE_CLIMB;
  }

  public void postClimb() {
    elbowSetpoint = SuperStructureConstants.kElbowPostClimbSetPoint;
    wristSetpoint = SuperStructureConstants.kWristPostClimbSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterPreClimbSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(POST_CLIMB)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.POST_CLIMB;
  }

  public void amp() {
    elbowSetpoint = SuperStructureConstants.kElbowAmpSetPoint;
    wristSetpoint = SuperStructureConstants.kWristAmpSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterAmpSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(AMP)");
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.AMP;
  }

  public void trap() {
    elbowSetpoint = SuperStructureConstants.kElbowTrapSetPoint;
    wristSetpoint = SuperStructureConstants.kWristTrapSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterTrapSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(TRAP)");
    flipMagazineOut = true;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.TRAP;
  }

  public void intake() {
    elbowSetpoint = SuperStructureConstants.kElbowIntakeSetPoint;
    wristSetpoint = SuperStructureConstants.kWristIntakeSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterIntakeSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    wristSubsystem.setPosition(wristSetpoint);

    logger.info("{} -> TRANSFER(INTAKE)");
    flipMagazineOut = false;
    curState = SuperStructureStates.TRANSFER;
    nextState = SuperStructureStates.INTAKE;
  }

  public void defense() {
    elbowSetpoint = SuperStructureConstants.kElbowDefenseSetPoint;
    wristSetpoint = SuperStructureConstants.kWristDefenseSetPoint;
    shooterSpeed = SuperStructureConstants.kShooterDefenseSetPoint;

    shooterSubsystem.setSpeed(shooterSpeed);
    elbowSubsystem.setPosition(elbowSetpoint);

    logger.info("{} -> TRANSFER(DEFENSE)");
    flipMagazineOut = true;
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
    }
  }
  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return null;
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
    DEFENSE
  }
}
