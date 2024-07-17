package frc.robot.constants;

public final class SuperStructureConstants {
  // STOW
  public static final double kWristStowSetPoint = -2345.0; // -2195
  public static final double kElbowStowSetPoint = 0.00122; // 0.14691
  public static final double kShooterStowSetPoint = 0.0;

  // SEQUENCE
  public static final double kElbowMinToMoveWrist = 0.04215;
  public static final double kWristMinToMoveElbow = -1532;

  // AMP
  public static final double kWristAmpSetPoint = -91.0;
  public static final double kElbowAmpSetPoint = 0.23375; // -0.07788 -13.5
  public static final double kShooterAmpSetPoint = 0.0;

  // PRE-CLIMB
  public static final double kWristPreClimbSetPoint = kWristStowSetPoint;
  public static final double kElbowPreClimbSetPoint = 0.38069;
  public static final double kShooterPreClimbSetPoint = 0.0;

  // TRAP
  public static final double kElbowFoldedSetPoint = 0.20111556; // -13.94
  public static final double kWristFoldedSetPoint = kWristStowSetPoint;
  public static final double kWristTrapSetPoint = 970.0; // 1020 1070 1120
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
  public static final double kShooterSubwooferSetPoint = 55; // 60

  // FEEDING
  public static final double kWristFeedingSetPoint = kWristIntakeSetPoint;
  public static final double kElbowFeedingSetPoint = kElbowSubwooferSetPoint;
  public static final double kLeftShooterFeedingSetPoint = 50; // 60
  public static final double kRightShooterFeedingSetPoint = 40;

  // EJECTING
  public static final double kWristEjectingSetPoint = kWristStowSetPoint; // -2245
  public static final double kElbowEjectingSetPoint = kElbowStowSetPoint; // 0.14691
  public static final double kShooterEjectingSetPoint = 8.0;

  // LOW FEEDING
  public static final double kWristLowFeedShotSetPoint = kWristIntakeSetPoint;
  public static final double kElbowLowFeedShotSetPoint = 0.13;
  public static final double kLeftShooterLowFeedShotSetPoint = 80;
  public static final double kRightShooterLowFeedShotSetPoint = 40;

  // HIGH FEEDING
  public static final double kWristHighFeedShotSetPoint = kWristIntakeSetPoint;
  public static final double kElbowHighFeedShotSetPoint = 0.02; // 0.03
  public static final double kLeftShooterHighFeedShotSetPoint = 85; // 77
  public static final double kRightShooterHighFeedShotSetPoint = 20; // 17

  // SOURCE INTAKE
  public static final double kWristSourceIntakeSetPoint = kWristIntakeSetPoint;
  public static final double kElbowSourceIntakeSetPoint = 0.22852;
  public static final double kLeftShooterSourceIntakeSetPoint = 0.0;
  public static final double kRightShooterSourceIntakeSetPoint = 0.0;

  // SPINUP
  public static final double kShooterSpinUpLeftSetPoint = 60;
  public static final double kShooterSpinUpRightSetPoint = 40;

  // SHOOT
  public static final double kWristShootSetPoint = kWristStowSetPoint;

  // DISRUPT
  public static final double kWristDisruptSetPoint = kWristStowSetPoint;
  public static final double kElbowDisruptSetPoint = kElbowStowSetPoint;
  public static final double kShooterDisruptSetPoint = 40.0;

  // Defense
  public static final double kElbowBlockSetPoint = 0.2144476;
  public static final double kWristBlockSetPoint = 798;
  public static final double kShooterBlockSetPoint = 0.0;

  // Auto Ignore Note
  public static final double kElbowIgnoreNoteSetPoint = kElbowSubwooferSetPoint;
  public static final double kBlueLeftShooterIgnoreNoteSetPoint = 10; // 10
  public static final double kBlueRightShooterIgnoreNoteSetPoint = 42; // 50
  public static final double kWristIgnoreNoteSetPoint = kWristSubwooferSetPoint;
}
