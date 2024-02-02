package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutonConstants {
  // field width (x) = 16.540988 m;
  // field height (y) = 8.21055 m;

  public final class Setpoints {
    // Starting Positions
    public static final Pose2d MI1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Wing Notes
    public static final Pose2d W1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W2 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W3 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Middle Notes
    public static final Pose2d M1 = new Pose2d(8.270494, 0.752856, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M2 = new Pose2d(8.270494, 2.429256, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M3 = new Pose2d(8.270494, 4.105275, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M4 = new Pose2d(8.270494, 5.781294, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M5 = new Pose2d(8.270494, 7.457694, Rotation2d.fromDegrees(0.0));

    // Shooting Positions
    public static final Pose2d AS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d MS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  }
}
