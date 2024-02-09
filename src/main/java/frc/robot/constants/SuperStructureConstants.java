package frc.robot.constants;

public final class SuperStructureConstants {
  // SEQUENCE
  public static final double kElbowMinToMoveWrist = 22.45;
  public static final double kWristMinToMoveElbow = -2860.0;

  // SHOOT
  public static final double kWristShootSetPoint = 0.0;

  // AMP
  public static final double kWristAmpSetPoint = -2469.0;
  public static final double kElbowAmpSetPoint = -17.5;
  public static final double kShooterAmpSetPoint = 0.0;

  // PRE-CLIMB
  public static final double kWristPreClimbSetPoint = 0.0;
  public static final double kElbowPreClimbSetPoint = 0.0;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kWristTrapSetPoint = 0.0;
  public static final double kElbowTrapSetPoint = 0.0;
  public static final double kShooterTrapSetPoint = 0.0;

  // POST-CLIMB
  public static final double kWristPostClimbSetPoint = 0.0;
  public static final double kElbowPostClimbSetPoint = 0.0;
  public static final double kShooterPostClimbSetPoint = 0.0;

  // INTAKE
  public static final double kWristIntakeSetPoint = -3102.0;
  public static final double kElbowIntakeSetPoint = 28.5;
  public static final double kShooterIntakeSetPoint = 0.0;

  // DEFENSE
  public static final double kWristDefenseSetPoint = 0.0;
  public static final double kElbowDefenseSetPoint = 0.0;
  public static final double kShooterDefenseSetPoint = 0.0;

  // PODIUM PREP
  public static final double kWristPodiumPrepSetPoint = 0.0;
  public static final double kElbowPodiumPrepSetPoint = 0.0;
  public static final double kShooterPodiumPrepSetPoint = 0.0;

  // PODIUM
  public static final double kWristPodiumSetPoint = 0.0;
  public static final double kElbowPodiumSetPoint = 0.0;
  public static final double kShooterPodiumSetPoint = 0.0;

  // SUBWOOFER
  public static final double kWristSubwooferSetPoint = kWristIntakeSetPoint;
  public static final double kElbowSubwooferSetPoint = kElbowIntakeSetPoint;
  public static final double kShooterSubwooferSetPoint = 80;

  // STOW
  public static final double kWristStowSetPoint = -3102.0;
  public static final double kElbowStowSetPoint = 34;
  public static final double kShooterStowSetPoint = 0.0;
}
