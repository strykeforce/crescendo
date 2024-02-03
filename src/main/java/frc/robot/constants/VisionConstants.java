package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final double kMaxTimeNoVision = 5;
  public static final int kResultsForWheels = 5;
  public static final double kTimeToDecayDev = 3;
  public static final double kStdDevDecayCoeff = -0.005;
  public static final double kMinStdDev = 0.01;
  public static final double kMaxAmbig = 0.15;
  public static final int kMaxTimesOffWheels = 5;

  // Velocity Filter
  public static final double kLinearCoeffOnVelFilter = 0.1;
  public static final double kOffsetOnVelFilter = 0.2;
  public static final double kSquaredCoeffOnVelFilter = 0.2;

  public static Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0));

  // Constants for cameras
  public static final int kNumCams = 2;

  // Names
  public static final String kCam1Name = "Front";
  public static final String kCam2Name = "Back";

  // Indexs
  public static final int kCam1Idx = 0;
  public static final int kCam2Idx = 0;

  // Poses
  public static final Pose3d kCam1Pose =
      new Pose3d(
          new Translation3d(0.2, 0.30, 0.58), new Rotation3d(0, Units.degreesToRadians(20.0), 0));
  public static final Pose3d kCam2Pose =
      new Pose3d(
          new Translation3d(0.25, 0.34, 0.44),
          new Rotation3d(0, Units.degreesToRadians(20.0), Units.degreesToRadians(15.0)));

  // Increase these numbers to trust sensor readings from encoders and gyros less. This matrix is
  // in the form [theta], with units in radians.
  public static Matrix<N1, N1> kLocalMeasurementStdDevs =
      VecBuilder.fill(Units.degreesToRadians(0.01));

  // Increase these numbers to trust global measurements from vision less. This matrix is in the
  // form [x, y, theta]ᵀ, with units in meters and radians.
  // Vision Odometry Standard devs
  public static Matrix<N3, N1> kVisionMeasurementStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5000));
}
