package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutonConstants {
  public static final double kDelayForPickup = 0.2;

  public static final int kSwitchStableCounts = 100;

  public static final String[][] kNonAmpPathMatrix = {
    {
      null,
      "NonAmpShoot1_MiddleNote1.toml",
      "NonAmpShoot1_MiddleNote2.toml",
      "NonAmpShoot1_MiddleNote3.toml",
      "NonAmpShoot1_MiddleNote4.toml",
      "NonAmpShoot1_MiddleNote5.toml"
    },
    {
      "MiddleNote1_NonAmpShoot.toml",
      null,
      "MiddleNote1_MiddleNote2.toml",
      "MiddleNote1_MiddleNote3.toml",
      "MiddleNote1_MiddleNote4.toml",
      "MiddleNote1_MiddleNote5.toml"
    },
    {
      "MiddleNote2_NonAmpShoot.toml",
      "MiddleNote2_MiddleNote1.toml",
      null,
      "MiddleNote2_MiddleNote3",
      "MiddleNote2_MiddleNote4.toml",
      "MiddleNote2_MiddleNote5.toml"
    },
    {
      "MiddleNote3_NonAmpShoot.toml",
      "MiddleNote3_MiddleNote1.toml",
      "MiddleNote3_MiddleNote2.toml",
      null,
      "MiddleNote3_MiddleNote4.toml",
      "MiddleNote3_MiddleNote5.toml"
    },
    {
      "MiddleNote4_NonAmpShoot.toml",
      "MiddleNote4_MiddleNote1.toml",
      "MiddleNote4_MiddleNote2.toml",
      "MiddleNote4_MiddleNote3.toml",
      null,
      "MiddleNote4_MiddleNote5.toml"
    },
    {
      "MiddleNote5_NonAmpShoot.toml",
      "MiddleNote5_MiddleNote1.toml",
      "MiddleNote5_MiddleNote2.toml",
      "MiddleNote5_MiddleNote3.toml",
      "MiddleNote5_MiddleNote4.toml",
      null
    }
  };

  public final class Setpoints {
    // Starting Positions
    public static final Pose2d MI1 =
        new Pose2d(1.47, 5.55, Rotation2d.fromDegrees(0.0)); // 1.34 on x
    public static final Pose2d AI1 = new Pose2d(1.09, 6.94, Rotation2d.fromDegrees(50));
    public static final Pose2d AI2 = new Pose2d(1.34, 6.275, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAI1 = new Pose2d(1.00, 4.19, Rotation2d.fromDegrees(-50));
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
    public static final Pose2d MS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAS1 = new Pose2d(4.2, 2.8, Rotation2d.fromDegrees(-33.2));
    public static final Pose2d NAS2 = new Pose2d(3.8, 5.1, Rotation2d.fromDegrees(-6.7));
  }

  // distances to speaker
  public static final double kAI1ToSpeakerDist = 1.2;
  public static final double kNAI1ToSpeakerDist =
      Math.hypot(Setpoints.NAI1.getX(), Setpoints.NAI1.getY() - RobotStateConstants.kSpeakerY);
}
