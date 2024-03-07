package frc.robot.constants;

public final class SuperStructureConstants {
  // STOW
  public static final double kWristStowSetPoint = -2245.0; // -2045
  public static final double kElbowStowSetPoint = 32.0; // 33.5
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
  public static final double kElbowPreClimbSetPoint = -49.0;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kElbowFoldedSetPoint = -13.94;
  public static final double kWristFoldedSetPoint = kWristStowSetPoint;
  public static final double kWristTrapSetPoint = 1040.0; // 585 800
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
  public static final double kWristDefenseSetPoint = 1513;
  public static final double kElbowDefenceIntermediateSetPoint = -10.0;
  public static final double kElbowDefenseFinalSetPoint = -20.0;
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
  public static final double kShooterSubwooferSetPoint = 60; // 80

  // SHOOT
  public static final double kWristShootSetPoint = kWristStowSetPoint;
}
