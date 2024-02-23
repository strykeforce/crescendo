package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutonConstants {
  public final class Setpoints {
    // Starting Positions
    public static final Pose2d MI1 = new Pose2d(1.34, 5.55, Rotation2d.fromDegrees(0.0));
    public static final Pose2d AI1 = new Pose2d(0.89, 6.58, Rotation2d.fromDegrees(60));
    public static final Pose2d AI2 = new Pose2d(1.34, 6.275, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAI1 = new Pose2d(0.96, 4.21, Rotation2d.fromDegrees(-60));

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
  }
}
