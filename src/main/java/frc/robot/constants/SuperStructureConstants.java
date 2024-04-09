package frc.robot.constants;

public final class SuperStructureConstants {
  // STOW
  public static final double kWristStowSetPoint = -2195.0; // -2245
  public static final double kElbowStowSetPoint = 0.00322; // 0.14691
  public static final double kShooterStowSetPoint = 0.0;

  // SEQUENCE
  public static final double kElbowMinToMoveWrist = 0.03915;
  public static final double kWristMinToMoveElbow = -1532;

  // AMP
  public static final double kWristAmpSetPoint = -91.0;
  public static final double kElbowAmpSetPoint = 0.23375; // -0.07788 -13.5
  public static final double kShooterAmpSetPoint = 0.0;

  // PRE-CLIMB
  public static final double kWristPreClimbSetPoint = kWristStowSetPoint;
  public static final double kElbowPreClimbSetPoint = 0.28069;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kElbowFoldedSetPoint = 0.19681; // -13.94
  public static final double kWristFoldedSetPoint = kWristStowSetPoint;
  public static final double kWristTrapSetPoint = 1120.0; // 585 800
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
  public static final double kElbowDefenceIntermediateSetPoint = kElbowStowSetPoint;
  public static final double kElbowDefenseFinalSetPoint = kElbowStowSetPoint;
  public static final double kShooterDefenseSetPoint = 0.0;

  // PODIUM PREP
  public static final double kWristPodiumPrepSetPoint = kWristStowSetPoint;
  public static final double kElbowPodiumPrepSetPoint = 0.26806; // -28.3
  public static final double kShooterPodiumPrepSetPoint = 0.05; // 1

  // PODIUM
  public static final double kWristPodiumSetPoint = kWristIntakeSetPoint;
  public static final double kElbowPodiumSetPoint = kElbowPodiumPrepSetPoint;
  public static final double kShooterPodiumSetPoint = -100;

  // SUBWOOFER
  public static final double kWristSubwooferSetPoint = kWristIntakeSetPoint;
  public static final double kElbowSubwooferSetPoint = kElbowIntakeSetPoint;
  public static final double kShooterSubwooferSetPoint = 50; // 60

  // FEEDING
  public static final double kWristFeedingSetPoint = kWristIntakeSetPoint;
  public static final double kElbowFeedingSetPoint = kElbowIntakeSetPoint;
  public static final double kLeftShooterFeedingSetPoint = 60;
  public static final double kRightShooterFeedingSetPoint = 40;

  // SPINUP
  public static final double kShooterSpinUpLeftSetPoint = 60;
  public static final double kShooterSpinUpRightSetPoint = 40;

  // SHOOT
  public static final double kWristShootSetPoint = kWristStowSetPoint;
}
