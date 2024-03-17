package frc.robot.constants;

public final class SuperStructureConstants {
  // STOW
  public static final double kWristStowSetPoint = -2245.0; // -2045
  public static final double kElbowStowSetPoint = 0.14691; // 0.151
  public static final double kShooterStowSetPoint = 0.0;

  // SEQUENCE
  public static final double kElbowMinToMoveWrist = 0.10776;
  public static final double kWristMinToMoveElbow = -1532;

  // AMP
  public static final double kWristAmpSetPoint = -91.0;
  public static final double kElbowAmpSetPoint = -0.07723; // -13.5
  public static final double kShooterAmpSetPoint = 0.0;

  // PRE-CLIMB
  public static final double kWristPreClimbSetPoint = kWristStowSetPoint;
  public static final double kElbowPreClimbSetPoint = -0.23520;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kElbowFoldedSetPoint = -0.05330; // -13.94
  public static final double kWristFoldedSetPoint = kWristStowSetPoint;
  public static final double kWristTrapSetPoint = 1070.0; // 585 800
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
  public static final double kElbowDefenceIntermediateSetPoint = -0.04800;
  public static final double kElbowDefenseFinalSetPoint = -0.09600;
  public static final double kShooterDefenseSetPoint = 0.0;

  // PODIUM PREP
  public static final double kWristPodiumPrepSetPoint = kWristStowSetPoint;
  public static final double kElbowPodiumPrepSetPoint = -0.12336; // -28.3
  public static final double kShooterPodiumPrepSetPoint = 0.05; // 1

  // PODIUM
  public static final double kWristPodiumSetPoint = kWristIntakeSetPoint;
  public static final double kElbowPodiumSetPoint = kElbowPodiumPrepSetPoint;
  public static final double kShooterPodiumSetPoint = -100;

  // SUBWOOFER
  public static final double kWristSubwooferSetPoint = kWristIntakeSetPoint;
  public static final double kElbowSubwooferSetPoint = kElbowIntakeSetPoint;
  public static final double kShooterSubwooferSetPoint = 60; // 80

  // SHOOT
  public static final double kWristShootSetPoint = kWristStowSetPoint;
}
