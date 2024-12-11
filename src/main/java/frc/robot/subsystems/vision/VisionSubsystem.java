package frc.robot.subsystems.vision;

import WallEye.UdpSubscriber;
import WallEye.WallEyeCam;
import WallEye.WallEyePoseResult;
import WallEye.WallEyeResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {

  // Private Variables
  private WallEyeCam[] cams;

  private Translation2d[] offsets = {
    VisionConstants.kCam1Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam2Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam3Pose.getTranslation().toTranslation2d(),
    VisionConstants.kCam4Pose.getTranslation().toTranslation2d()
  };

  private Rotation2d[] rotsOff = {
    VisionConstants.kCam1Pose.getRotation().toRotation2d(),
    VisionConstants.kCam2Pose.getRotation().toRotation2d(),
    VisionConstants.kCam3Pose.getRotation().toRotation2d(),
    VisionConstants.kCam4Pose.getRotation().toRotation2d()
  };

  private String[] names = {
    VisionConstants.kCam1Name,
    VisionConstants.kCam2Name,
    VisionConstants.kCam3Name,
    VisionConstants.kCam4Name
  };

  private String[] Pinames = {
    VisionConstants.kPi1Name,
    VisionConstants.kPi2Name,
    VisionConstants.kPi3Name,
    VisionConstants.kPi3Name
  };

  private int[] camIndex = {
    VisionConstants.kCam1Idx,
    VisionConstants.kCam2Idx,
    VisionConstants.kCam3Idx,
    VisionConstants.kCam4Idx
  };

  private UdpSubscriber[] udps;

  private ArrayList<Pair<WallEyeResult, Integer>> validResults =
      new ArrayList<>(); // <Result, Cam #>
  private boolean visionUpdates = true;
  private double timeLastVision = 0;
  private int updatesToWheels = 0;
  private DriveSubsystem driveSubsystem;
  private int offWheels = 0;
  private Logger logger;
  private int minTagsNeeded = 2;

  private AprilTagFieldLayout field;

  private CircularBuffer<Double> gyroData =
      new CircularBuffer<Double>(VisionConstants.kCircularBufferSize);

  // Deadeye<TargetListTargetData> cam = new Deadeye<TargetListTargetData>("A0",
  // TargetListTargetData.class, NetworkTableInstance.getDefault(), null);

  private Matrix<N3, N1> adaptiveVisionMatrix;
  private Matrix<N3, N1> scaledStdDev;

  private double fedStdDevs = 0.0;
  private RobotStateSubsystem robotStateSubsystem;

  // Constructor
  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    logger = LoggerFactory.getLogger(this.getClass());

    cams = new WallEyeCam[VisionConstants.kNumCams];
    udps = new UdpSubscriber[VisionConstants.kNumPis];
    adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
    scaledStdDev = adaptiveVisionMatrix.copy();
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      logger.error("COULD NOT PARSE APRILTAG LAYOUT");
    }

    // Initialize all walleyecams
    for (int i = 0; i < VisionConstants.kNumCams; ++i) {
      cams[i] = new WallEyeCam(Pinames[i], camIndex[i], -1);
    }

    // Start monitoring for UDP data
    udps[0] = new UdpSubscriber(VisionConstants.kUdpPorts[0], cams[0]);
    udps[1] = new UdpSubscriber(VisionConstants.kUdpPorts[1], cams[1]);
    udps[2] = new UdpSubscriber(VisionConstants.kUdpPorts[2], cams[2], cams[3]);
  }

  // Getter/Setter Methods
  public void setVisionUpdates(boolean visionUpdates) {
    this.visionUpdates = visionUpdates;
  }

  public void setRobotStateSubsystem(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  public void setMinTagsNeeded(int num) {
    minTagsNeeded = num;
  }

  public boolean isVisionUpdatingDrive() {
    return visionUpdates;
  }

  public boolean isCameraConnected(int index) {
    if (index == 1) index = 0; // FIXME
    return cams[index].isCameraConnected();
  }

  // Helper Methods

  private double getSeconds() {
    return RobotController.getFPGATime() / 1000000.0;
  }

  private boolean isPoseValidWithWheels(WallEyePoseResult test, Translation2d pose) {
    if (isPoseValidWithoutWheels(test, pose)) {

      // Get Speed and current position
      ChassisSpeeds speed = driveSubsystem.getFieldRelSpeed();
      Pose2d curPose = driveSubsystem.getPoseMeters();

      // Find the x and y difference between cam and wheels
      Translation2d disp = (curPose.getTranslation().minus(pose));
      double magnitudeVel =
          Math.sqrt(Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2));
      double magnitudeDisp = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

      // Test to see if the displacement falls beneath a line based on velocity
      return test.getNumTags() >= minTagsNeeded
          && (magnitudeDisp
              < ((magnitudeVel * VisionConstants.kLinearCoeffOnVelFilter)
                  + VisionConstants.kOffsetOnVelFilter
                  + Math.pow((magnitudeVel * VisionConstants.kSquaredCoeffOnVelFilter), 2)));
    }
    return false;
  }

  private boolean isPoseValidWithoutWheels(WallEyePoseResult test, Translation2d location) {
    return (test.getNumTags() >= 2 || test.getAmbiguity() <= VisionConstants.kMaxAmbig)
        && (location.getX() < DriveConstants.kFieldMaxX && location.getX() > 0)
        && (location.getY() < DriveConstants.kFieldMaxY && location.getY() > 0);
  }

  private double minTagDistance(WallEyePoseResult rst) {
    Translation2d loc = rst.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = rst.getTagIDs();
    double minDistance = 2767;
    for (int id : ids) {
      Translation2d tagLoc = field.getTagPose(id).get().getTranslation().toTranslation2d();
      double dist = loc.getDistance(tagLoc);
      if (dist < minDistance) {
        minDistance = dist;
      }
    }
    return minDistance;
  }

  private double avgTagDistance(WallEyePoseResult rst) {
    Translation2d loc = rst.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = rst.getTagIDs();
    double avg = 0.0;
    int n = ids.length;
    for (int id : ids) {
      Translation2d tagLoc = field.getTagPose(id).get().getTranslation().toTranslation2d();
      double dist = loc.getDistance(tagLoc);
      avg += dist * 1.0 / n;
    }
    return avg;
  }

  private double getStdDevFactor(double distance, int numTags, String camName) {
    switch (camName) {
      case "AngledShooterLeft":
      case "AngledShooterRight":
        if (numTags == 1)
          return 1
              / (3
                  * FastMath.pow(
                      VisionConstants.baseNumber,
                      FastMath.pow(
                          VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                          VisionConstants.FOV58MJPGPowerNumber)));
        return 1
            / FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58MJPGMultiTagCoeff * distance,
                    VisionConstants.FOV58MJPGPowerNumber));
      case "Shooter":
        if (numTags == 1)
          return 1
              / FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV58YUYVSingleTagCoeff * distance,
                      VisionConstants.FOV58YUYVPowerNumber));
        return 1
            / FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58YUYVMultiTagCoeff * distance,
                    VisionConstants.FOV58YUYVPowerNumber));
      case "Intake":
        if (numTags == 1)
          return 1
              / FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV45SinlgeTagCoeff * distance,
                      VisionConstants.FOV45powerNumber));
        return 1
            / FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV45MultiTagCoeff * distance,
                    VisionConstants.FOV45powerNumber));

      default:
        if (numTags == 1)
          return 1
              / FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.singleTagCoeff * distance, VisionConstants.powerNumber));
        return 1
            / FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.multiTagCoeff * distance, VisionConstants.powerNumber));
    }
  }

  private Pose2d getCloserPose(Pose2d pose1, Pose2d pose2, double rotation) {
    if (Math.abs(new Rotation2d(rotation).minus(pose1.getRotation()).getRadians())
        <= Math.abs(new Rotation2d(rotation).minus(pose2.getRotation()).getRadians())) return pose1;
    else return pose2;
  }

  private Pose2d getCorrectPose(Pose2d pose1, Pose2d pose2, double timestamp) {
    if (gyroData.size() != VisionConstants.kCircularBufferSize) return pose1;
    double rotation =
        gyroData.get(FastMath.floorToInt((timestamp / 1_000_000.0) / VisionConstants.kLoopTime));
    return getCloserPose(pose1, pose2, rotation);
  }

  // Periodic
  @Override
  public void periodic() {
    double gyroBuffer =
        FastMath.normalizeMinusPiPi(driveSubsystem.getGyroRotation2d().getRadians());

    // double gyroBuffer =
    // FastMath.normalizeMinusPiPi(driveSubsystem.getPoseMeters().getRotation().getRadians());
    gyroData.addFirst(gyroBuffer);
    org.littletonrobotics.junction.Logger.recordOutput("VisionSubsystem/gyroBuffer", gyroBuffer);

    scaledStdDev = adaptiveVisionMatrix.copy();

    // If enough time elapses between camera updates - reset count of updates to 0
    if ((getSeconds() - timeLastVision > VisionConstants.kTimeToResetWheelCount)
        && updatesToWheels >= 1) {
      logger.info("Reset # of Vision updates to 0");
      updatesToWheels = 0;
    }

    // Clear out old results
    validResults.clear();

    // Go through each camera and store result and array idx in pair
    for (int i = 0; i < VisionConstants.kNumCams; ++i) {
      org.littletonrobotics.junction.Logger.recordOutput(
          "VisionSubsystem/UpdateCount " + names[i], cams[i].getUpdateNumber());

      if (cams[i].hasNewUpdate()) {
        timeLastVision = getSeconds();
        validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));

        org.littletonrobotics.junction.Logger.recordOutput(
            "VisionSubsystem/UpdateTime " + names[i], cams[i].getResults().getTimeStamp());
      }
    }
    // Tightens std devs if time elapses
    if (getSeconds() - timeLastVision >= VisionConstants.kTimeToDecayDev) {

      // Take x and y weights and linearly decrease them
      for (int i = 0; i < 2; ++i) {
        double estimatedWeight =
            VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                + VisionConstants.kStdDevDecayCoeff
                    * (getSeconds() - timeLastVision - VisionConstants.kTimeToDecayDev);
        adaptiveVisionMatrix.set(
            i,
            0,
            estimatedWeight > VisionConstants.kMinStdDev
                ? estimatedWeight
                : VisionConstants.kMinStdDev);
      }
    }
    // Go through results
    for (Pair<WallEyeResult, Integer> res : validResults) {
      if (res.getFirst() instanceof WallEyePoseResult) {
        adaptiveVisionMatrix.set(0, 0, .1);
        adaptiveVisionMatrix.set(1, 0, .1);

        // Take out data from pair
        WallEyePoseResult result = (WallEyePoseResult) res.getFirst();
        int idx = res.getSecond();

        for (int i = 0; i < 2; ++i)
          if (result.getNumTags() > 0)
            scaledStdDev.set(
                i,
                0,
                adaptiveVisionMatrix.get(0, 0)
                    / getStdDevFactor(
                        result.getNumTags() == 1 ? minTagDistance(result) : avgTagDistance(result),
                        result.getNumTags(),
                        names[idx]));

        Pose2d cameraPose;
        Translation2d centerPos;
        Rotation2d cameraRot;

        // Get center of Robot pose
        if (result.getNumTags() > 1) {
          cameraPose = result.getCameraPose().toPose2d();

          centerPos =
              cameraPose
                  .getTranslation()
                  .minus(offsets[idx].rotateBy(cameraPose.getRotation().rotateBy(rotsOff[idx])));

          cameraRot = cameraPose.getRotation().rotateBy(rotsOff[idx]);
        } else {
          Pose2d cameraPose1 = result.getFirstPose().toPose2d();
          Pose2d cameraPose2 = result.getSecondPose().toPose2d();

          cameraPose1 =
              new Pose2d(
                  cameraPose1.getTranslation(), cameraPose1.getRotation().rotateBy(rotsOff[idx]));
          cameraPose2 =
              new Pose2d(
                  cameraPose2.getTranslation(), cameraPose2.getRotation().rotateBy(rotsOff[idx]));

          String outputString = "VisionSubsystem/Pose" + names[idx] + "1";
          org.littletonrobotics.junction.Logger.recordOutput(outputString, cameraPose1);
          outputString = "VisionSubsystem/Pose" + names[idx] + "2";
          org.littletonrobotics.junction.Logger.recordOutput(outputString, cameraPose2);

          cameraPose = getCorrectPose(cameraPose1, cameraPose2, result.getTimeStamp());

          centerPos =
              cameraPose.getTranslation().minus(offsets[idx].rotateBy(cameraPose.getRotation()));
          cameraRot = cameraPose.getRotation();
        }
        if (isPoseValidWithoutWheels(result, centerPos)) {
          String outputAccept = "VisionSubsystem/AcceptedCam" + names[idx] + "Pose";
          org.littletonrobotics.junction.Logger.recordOutput(
              outputAccept, new Pose2d(centerPos, cameraRot));

          String rawCamera = "VisionSubsystem/RawAcceptedCam" + names[idx] + "Pose";
          org.littletonrobotics.junction.Logger.recordOutput(
              rawCamera, result.getCameraPose().toPose2d());
          updatesToWheels++;

          fedStdDevs = scaledStdDev.get(0, 0);

          // if (fedStdDevs < VisionConstants.kThetaStdThres && result.getNumTags() >= 2)
          // {
          // scaledStdDev.set(2, 0, VisionConstants.kThetaStdDevUsed);
          // } else {
          // scaledStdDev.set(2, 0, VisionConstants.kThetaStdDevRejected);
          // }

          if (visionUpdates) {
            driveSubsystem.addVisionMeasurement(
                new Pose2d(centerPos, cameraRot), result.getTimeStamp() / 1_000_000, scaledStdDev);
          }

        } else {
          String output = "VisionSubsystem/NotAcceptedCam" + names[idx] + "Pose";
          org.littletonrobotics.junction.Logger.recordOutput(
              output, new Pose2d(centerPos, cameraRot));

          String rawCamera = "VisionSubsystem/RawNotAcceptedCam" + names[idx] + "Pose";
          org.littletonrobotics.junction.Logger.recordOutput(
              rawCamera, result.getCameraPose().toPose2d());
        }
      }
    }
  }

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        // new Measure("State", () -> curState.ordinal()),
        new Measure("OffWheels", () -> offWheels),
        new Measure("Updates To Wheels", () -> updatesToWheels),
        new Measure("Adaptive Vision Matrix", () -> adaptiveVisionMatrix.get(0, 0)),
        new Measure("fedStdDevs", () -> fedStdDevs));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
