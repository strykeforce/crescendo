package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutonConstants {
  public final class Setpoints {
    // Starting Positions
    public static final Pose2d MI1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAI1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Wing Notes
    public static final Pose2d W1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W2 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d W3 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Middle Notes
    public static final Pose2d M1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M2 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M3 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M4 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d M5 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Shooting Positions
    public static final Pose2d AS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d MS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d NAS1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  }
}
