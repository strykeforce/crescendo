package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.telemetry.TelemetryService;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public double odometryX = 0.0;
    public double odometryY = 0.0;
    public double odometryRotation2D = 0.0;
    public Rotation2d gyroRotation2d = new Rotation2d();
    public double gyroPitch = 0.0;
    public double gyroRoll = 0.0;
    public double gyroRate = 0.0;
    public boolean isConnected = false;
    public Pose2d poseMeters = new Pose2d();
    public double updateCount = 0;
    public double[] azimuthVels = {0, 0, 0, 0};
    public double[] azimuthCurrent = {0, 0, 0, 0};
  }

  public default SwerveModule[] getSwerveModules() {
    return null;
  }

  public default SwerveModulePosition[] getSwerveModulePositions() {
    return null;
  }

  public default SwerveModuleState[] getSwerveModuleStates() {
    return null;
  }

  public default ChassisSpeeds getFieldRelSpeed() {
    return null;
  }

  public default SwerveDriveKinematics getKinematics() {
    return null;
  }

  public default void setOdometry(Rotation2d Odom) {}

  public default void setGyroOffset(Rotation2d rotation) {}

  public default void resetGyro() {}

  public default void updateSwerve() {}

  public default void resetOdometry(Pose2d pose) {}

  public default void addVisionMeasurement(Pose2d pose, double timestamp) {}

  public default void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public default void drive(
      double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {}

  public default void move(
      double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {}

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default BooleanSupplier getAzimuth1FwdLimitSwitch() {
    return () -> false;
  }
}
