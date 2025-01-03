package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import net.jafama.FastMath;

public final class AutonConstants {
  public static final double kDelayForPickup = 0.15; // 0.2 -> 0.1 -> 0.15
  public static final double kDelayForDeadeye = 1.0;
  public static final double kPDeadEyeYDrive = 0.03; // 0.02
  public static final double kIDeadEyeYDrive = 0.0;
  public static final double kDDeadEyeYDrive = 0.0;
  public static final double kPDeadEyeXDrive = 0.03;
  public static final double kIDeadEyeXDrive = 0.0;
  public static final double kDDeadEyeXDrive = 0.0;
  public static final double kMaxVelDeadeyeDrive = 4.0;
  public static final double kMaxAutonVelDeadeyeDrive = 2.0;
  public static final double kMaxAccelDeadeyeDrive = 3.5;
  public static final double kXSpeed = 2.0;
  public static final double kMaxXOff = 1.93; // 1.0
  public static final double kSwitchXLine = 6.5;
  public static final double kWingSwitchXLine = 1.4;
  public static final double kPercentLeft = 0.65;
  public static final double kTerminateWithNotePercentLeft = 0.8;
  public static final double kWingPercentLeft = 0.5;
  public static final double kDeadeyeHuntOmegaRadps = -1.5;
  public static final double kDeadeyeHuntStartYawRads = FastMath.toRadians(-7);
  public static final double kFoundNoteLoopCounts = 3;

  public static final int kSwitchStableCounts = 100;

  public static final double kForwardVel = 1.5;

  public static final String[][] kNonAmpPathMatrix = {
    {
      null,
      "NonAmpShoot2_MiddleNote1",
      "NonAmpShoot2_MiddleNote2",
      "NonAmpShoot2_MiddleNote3",
      "NonAmpShoot2_MiddleNote4_B",
      "NonAmpShoot2_MiddleNote5"
    },
    {
      "MiddleNote1_NonAmpShoot2",
      null,
      "MiddleNote1_MiddleNote2",
      "MiddleNote1_MiddleNote3",
      "MiddleNote1_MiddleNote4",
      "MiddleNote1_MiddleNote5"
    },
    {
      "MiddleNote2_NonAmpShoot2",
      "MiddleNote2_MiddleNote1",
      null,
      "MiddleNote2_MiddleNote3",
      "MiddleNote2_MiddleNote4",
      "MiddleNote2_MiddleNote5"
    },
    {
      "MiddleNote3_NonAmpShoot2",
      "MiddleNote3_MiddleNote1",
      "MiddleNote3_MiddleNote2",
      null,
      "MiddleNote3_MiddleNote4",
      "MiddleNote3_MiddleNote5"
    },
    {
      "MiddleNote4_NonAmpShoot2_B",
      "MiddleNote4_MiddleNote1",
      "MiddleNote4_MiddleNote2",
      "MiddleNote4_MiddleNote3",
      null,
      "MiddleNote4_MiddleNote5"
    },
    {
      "MiddleNote5_NonAmpShoot2",
      "MiddleNote5_MiddleNote1",
      "MiddleNote5_MiddleNote2",
      "MiddleNote5_MiddleNote3",
      "MiddleNote5_MiddleNote4",
      null
    }
  };

  public static final String[][] kAmpPathMatrix = {
    {
      null, "AmpShoot2_MiddleNote1", "AmpShoot2_MiddleNote2", "AmpShoot2_MiddleNote3", null, null,
    },
    {
      "MiddleNote1_AmpShoot2",
      null,
      "MiddleNote1_MiddleNote2",
      "MiddleNote1_MiddleNote3",
      "MiddleNote1_MiddleNote4",
      null,
      null
    },
    {
      "MiddleNote2_AmpShoot2",
      "MiddleNote2_MiddleNote1",
      null,
      "MiddleNote2_MiddleNote3",
      null,
      null,
    },
    {
      "MiddleNote3_AmpShoot2",
      "MiddleNote3_MiddleNote1",
      "MiddleNote3_MiddleNote2",
      null,
      "MiddleNote3_MiddleNote4",
      "MiddleNote3_MiddleNote5"
    },
    {
      null, null, null, null, null,
    },
    {null, null, null, null, null, null}
  };

  public static final String[][] kNonAmpSpecialPathMatrix = {
    {null, null, null, null, "NonAmpShoot4_MiddleNote4", "NonAmpShoot4_MiddleNote5"},
    {
      null,
      null,
      "MiddleNote1_MiddleNote2",
      "MiddleNote1_MiddleNote3",
      "MiddleNote1_MiddleNote4",
      "MiddleNote1_MiddleNote5"
    },
    {
      null,
      "MiddleNote2_MiddleNote1",
      null,
      "MiddleNote2_MiddleNote3",
      "MiddleNote2_MiddleNote4",
      "MiddleNote2_MiddleNote5"
    },
    {
      null,
      "MiddleNote3_MiddleNote1",
      "MiddleNote3_MiddleNote2",
      null,
      "MiddleNote3_MiddleNote4",
      "MiddleNote3_MiddleNote5"
    },
    {
      "MiddleNote4_NonAmpShoot4",
      "MiddleNote4_MiddleNote1",
      "MiddleNote4_MiddleNote2",
      "MiddleNote4_MiddleNote3",
      null,
      "MiddleNote4_MiddleNote5"
    },
    {
      "MiddleNote5_NonAmpShoot4",
      "MiddleNote5_MiddleNote1",
      "MiddleNote5_MiddleNote2",
      "MiddleNote5_MiddleNote3",
      "MiddleNote5_MiddleNote4",
      null
    }
  };

  public final class Setpoints {
    // Starting Positions
    public static final Pose2d MI1 =
        new Pose2d(1.47, 5.55, Rotation2d.fromDegrees(0.0)); // 1.34 on x
    public static final Pose2d MI2 = new Pose2d(1.47, 4.905, Rotation2d.fromDegrees(0.0));
    public static final Pose2d AI1 = new Pose2d(1.09, 6.94, Rotation2d.fromDegrees(50));
    public static final Pose2d AI2 = new Pose2d(1.34, 6.275, Rotation2d.fromDegrees(0.0));
    public static final Pose2d AI3 = new Pose2d(1.43764, 7.33425, Rotation2d.fromDegrees(90.0));
    public static final Pose2d AI4 =
        new Pose2d(
            kAutonLineX - DriveConstants.kRobotWidth,
            DriveConstants.kFieldMaxY - kAmpZoneWidth - DriveConstants.kRobotLength,
            Rotation2d.fromDegrees(-90));
    public static final Pose2d NAI1 = new Pose2d(1.00, 4.19, Rotation2d.fromDegrees(-50));
    public static final Pose2d NAI2 =
        new Pose2d(
            1.93294 - DriveConstants.kRobotLength / 2 - 0.13,
            4.11 - DriveConstants.kRobotWidth - 0.13,
            Rotation2d.fromDegrees(0));
    // 0.96 4.156

    // Wing Notes
    public static final Pose2d W1 = new Pose2d(2.89, 7.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W2 = new Pose2d(2.89, 5.55, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W3 = new Pose2d(2.89, 4.11, Rotation2d.fromDegrees(0.0));

    // Middle Notes
    public static final Pose2d M1 = new Pose2d(8.270494, 7.457694, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M2 = new Pose2d(8.270494, 5.781294, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M3 = new Pose2d(8.270494, 4.105275, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M4 = new Pose2d(8.270494, 2.429256, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M5 = new Pose2d(8.270494, 0.752856, Rotation2d.fromDegrees(0.0));

    // Shooting Positions
    public static final Pose2d AS1 = new Pose2d(3.89, 5.55, Rotation2d.fromDegrees(0.0));
    public static final Pose2d AS2 = new Pose2d(4.3, 6.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d AS3 = new Pose2d(1.43764, 7.33425, Rotation2d.fromDegrees(90.0));
    public static final Pose2d MS1 = new Pose2d(4.3, 5.55, Rotation2d.fromDegrees(0.0));
    public static final Pose2d MS2 = new Pose2d(4.3, 6.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d MS3 =
        MI1; // new Pose2d(1.57, 4.737, Rotation2d.fromDegrees(0.0)); // y = 5.05
    public static final Pose2d NAS1 = new Pose2d(4.2, 2.8, Rotation2d.fromDegrees(-33.2));
    public static final Pose2d NAS2 =
        new Pose2d(4.4, 4.6, Rotation2d.fromDegrees(-12.2)); // 4.55, 4.6 -12.2 -> 4,5.1, -6.7
    public static final Pose2d NAS2_SPIN =
        new Pose2d(4.36, 4.74, Rotation2d.fromDegrees(-12.2)); // 4,5.1, -6.7
    public static final Pose2d NAS3 =
        new Pose2d(3.052, 3.063, Rotation2d.fromDegrees(-47.39)); // Disrupt auto
    public static final Pose2d NAS4 = new Pose2d(2.0, 4.11, Rotation2d.fromDegrees(144.25));
    public static final Pose2d shooterPrepNAS4 =
        new Pose2d(1.58, 4.55, Rotation2d.fromDegrees(144.25));

    // Path Midpoints
    public static final Pose2d MP1 = new Pose2d(2.7, 7.67, Rotation2d.fromDegrees(0.0)); // 5,7.61
    public static final Pose2d MP2 = new Pose2d(3.89, 7.2, Rotation2d.fromDegrees(-90)); // AS4
  }

  // distances to speaker
  public static final double kAI1ToSpeakerDist = 1.2;
  public static final double kNAI1ToSpeakerDist =
      Math.hypot(Setpoints.NAI1.getX(), Setpoints.NAI1.getY() - RobotStateConstants.kSpeakerY);
  public static final double kAutonLineX = 1.93294;
  public static final double kAmpZoneWidth = 0.45085;

  public static final double kDisruptIntakingMiddleNote1Y = 7.0;
  public static final double kDisruptIntakingMiddleNote2Y = 5.2;
  public static final double kHuntStartAngle = FastMath.toRadians(-160);
  public static final double kHuntEndAngle = FastMath.toRadians(60);
  public static final double kHuntTurnSpeed = 2.0;
  public static final double kRotateAngleSpeed = 4.0;
}
