package frc.robot.constants;

public final class SuperStructureConstants {
  // STOW
  public static final double kWristStowSetPoint = -2045.0;
  public static final double kElbowStowSetPoint = 33.5;
  public static final double kShooterStowSetPoint = 0.0;

  // SEQUENCE
  public static final double kElbowMinToMoveWrist = 22.45;
  public static final double kWristMinToMoveElbow = -1532;

  // AMP
  public static final double kWristAmpSetPoint = -91.0;
  public static final double kElbowAmpSetPoint = -16.09; // -13.5
  public static final double kShooterAmpSetPoint = 0.0;

  // PRE-CLIMB
  public static final double kWristPreClimbSetPoint = kWristStowSetPoint;
  public static final double kElbowPreClimbSetPoint = -30.0;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kElbowFoldedSetPoint = 0.0;
  public static final double kWristFoldedSetPoint = kWristStowSetPoint;
  public static final double kWristTrapSetPoint = 0.0;
  public static final double kElbowTrapSetPoint = kElbowFoldedSetPoint;
  public static final double kShooterTrapSetPoint = 0.0;

  // POST-CLIMB
  public static final double kWristPostClimbSetPoint = kWristStowSetPoint;
  public static final double kElbowPostClimbSetPoint = kElbowFoldedSetPoint;
  public static final double kShooterPostClimbSetPoint = 0.0;

  // INTAKE
  public static final double kWristIntakeSetPoint = kWristStowSetPoint;
  public static final double kElbowIntakeSetPoint = kElbowStowSetPoint;
  public static final double kShooterIntakeSetPoint = 0.0;

  // DEFENSE
  public static final double kWristDefenseSetPoint = kWristStowSetPoint;
  public static final double kElbowDefenseSetPoint = 0.0;
  public static final double kShooterDefenseSetPoint = 0.0;

  // PODIUM PREP
  public static final double kWristPodiumPrepSetPoint = kWristStowSetPoint;
  public static final double kElbowPodiumPrepSetPoint = -23.2;
  public static final double kShooterPodiumPrepSetPoint = 1;

  // PODIUM
  public static final double kWristPodiumSetPoint = kWristIntakeSetPoint;
  public static final double kElbowPodiumSetPoint = kElbowPodiumPrepSetPoint;
  public static final double kShooterPodiumSetPoint = -30;

  // SUBWOOFER
  public static final double kWristSubwooferSetPoint = kWristIntakeSetPoint;
  public static final double kElbowSubwooferSetPoint = kElbowIntakeSetPoint;
  public static final double kShooterSubwooferSetPoint = 80;

  // SHOOT
  public static final double kWristShootSetPoint = kWristStowSetPoint;
}
