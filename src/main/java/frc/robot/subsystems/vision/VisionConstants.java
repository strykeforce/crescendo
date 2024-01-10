package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final int kNumCams = 4;
  public static final long kMaxTimeNoVision = 5 * 1000000;
  public static final int kResultsForWheels = 5;
  public static final long kTimeToDecayDev = 3 * 1000000;
  public static final double kStdDevDecayCoeff = -0.1;
  public static final double kMinStdDev = 0.01;

  // Increase these numbers to trust global measurements from vision less. This matrix is in the
  // form [x, y, theta]ᵀ, with units in meters and radians.
  // Vision Odometry Standard devs
  public static Matrix<N3, N1> kVisionMeasurementStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5000));
}
