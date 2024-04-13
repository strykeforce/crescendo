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
  public static final double kMaxTimeNoVision = 20;
  public static final double kTimeToResetWheelCount = 5;
  public static final int kResultsForWheels = 5;
  public static final double kTimeToDecayDev = 10;
  public static final double kStdDevDecayCoeff = -0.005;
  public static final double kMinStdDev = 0.01;
  public static final double kMaxAmbig = 1.0;
  public static final int kMaxTimesOffWheels = 5;
  public static final double kBumperPixelLine = 87; // 100

  // Velocity Filter
  public static final double kLinearCoeffOnVelFilter = 0.1;
  public static final double kOffsetOnVelFilter = 0.10;
  public static final double kSquaredCoeffOnVelFilter = 0.1;

  public static Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0));

  public static final double kTimeStampOffset = 0.0;

  // StdDev scaling
  public static final double singleTagCoeff = 25.0 / 100.0;
  public static final double multiTagCoeff = 18.0 / 100.0;
  public static final double baseNumber = Math.E;
  public static final double powerNumber = 4.0;
  public static final double FOV45MultiTagCoeff = 16.0 / 100.0;
  public static final double FOV45powerNumber = 4.5;
  public static final double FOV45SinlgeTagCoeff = 21.0 / 100.0;
  public static final double FOV58MJPGMultiTagCoeff = 16.0 / 100.0;
  public static final double FOV58MJPGPowerNumber = 3.5;
  public static final double FOV58MJPGSingleTagCoeff = 21.0 / 100.0;
  public static final double FOV58YUYVMultiTagCoeff = 15.0 / 100.0;
  public static final double FOV58YUYVPowerNumber = 3.5;
  public static final double FOV58YUYVSingleTagCoeff = 19.0 / 100.0;

  // Constants for cameras
  public static final int kNumCams = 4;

  // Names
  public static final String kCam1Name = "Shooter";
  public static final String kCam2Name = "Intake";
  public static final String kCam3Name = "AngledShooterLeft";
  public static final String kCam4Name = "AngledShooterRight";

  public static final String kPi1Name = "Shooter";
  public static final String kPi2Name = "Intake";
  public static final String kPi3Name = "AngledShooters";

  // Indexs
  public static final int kCam1Idx = 0;
  public static final int kCam2Idx = 0;
  public static final int kCam3Idx = 0;
  public static final int kCam4Idx = 1;

  public static final double kLoopTime = 0.02;
  public static final int kCircularBufferSize = 1000;
  // Poses
  public static final Pose3d kCam1Pose =
      new Pose3d(
          new Translation3d(-0.27, 0.055, 0.20),
          new Rotation3d(0, Units.degreesToRadians(20.0), Units.degreesToRadians(180.0)));
  public static final Pose3d kCam2Pose =
      new Pose3d(
          new Translation3d(-0.21, -0.31, 0.44),
          new Rotation3d(0, Units.degreesToRadians(20.0), Units.degreesToRadians(0.0)));
  public static final Pose3d kCam3Pose =
      new Pose3d(
          new Translation3d(-0.23, 0.33, 0.56),
          new Rotation3d(0, Units.degreesToRadians(20.0), Units.degreesToRadians(-132.0)));
  public static final Pose3d kCam4Pose =
      new Pose3d(
          new Translation3d(-0.22, -0.335, 0.50),
          new Rotation3d(0, Units.degreesToRadians(20.0), Units.degreesToRadians(138.0)));

  // Increase these numbers to trust sensor readings from encoders and gyros less. This matrix is
  // in the form [theta], with units in radians.
  public static Matrix<N1, N1> kLocalMeasurementStdDevs =
      VecBuilder.fill(Units.degreesToRadians(0.01));

  // Increase these numbers to trust global measurements from vision less. This matrix is in the
  // form [x, y, theta]ᵀ, with units in meters and radians.
  // Vision Odometry Standard devs
  public static Matrix<N3, N1> kVisionMeasurementStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(360));
}
