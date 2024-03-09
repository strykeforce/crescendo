package frc.robot.constants;

public final class RobotStateConstants {
  public static final String kLookupTablePath = "/home/lvuser/deploy/LookupTable.csv";
  public static final double kLookupMinDistance = 1.5; // 150cm
  public static final double kLookupMaxDistance = 5.4; // 540cm
  public static final double kDistanceIncrement = 0.01; // 1cm
  public static final double kDistanceOffset = 0.00;
  public static final double kElbowShootOffset = 1.2; // .7

  public static final double kSpeakerY = 5.547868; // m

  public static final double kPodiumReleaseTime = 1.0;

  public static final double kTimeToStowPostAmp = 0.1;

  public static final double kClimbMoveElbowPos = 17; // 10

  public static final double kMinWristToMoveTrapBar = -1550;
  public static final double kMaxWristToMoveTrapBar = 0;
  public static final double kTrapTimer = 1.0;
  public static final double kClimbTrapTimer = 1.5; // 0.5
}
