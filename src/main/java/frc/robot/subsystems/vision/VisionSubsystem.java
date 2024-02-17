package frc.robot.subsystems.vision;

import WallEye.WallEyeCam;
import WallEye.WallEyeResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {

  // Private Variables
  WallEyeCam[] cams;

  Translation3d[] offsets = {
    VisionConstants.kCam1Pose.getTranslation(), VisionConstants.kCam2Pose.getTranslation()
  };

  Rotation3d[] rotsOff = {
    VisionConstants.kCam1Pose.getRotation(), VisionConstants.kCam2Pose.getRotation()
  };

  String[] names = {VisionConstants.kCam1Name, VisionConstants.kCam2Name};

  int[] camIndex = {VisionConstants.kCam1Idx, VisionConstants.kCam2Idx};

  ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>(); // <Result, Cam #>
  VisionStates curState = VisionStates.TRUSTWHEELS;
  boolean visionUpdates = true;
  double timeLastVision = 0;
  int updatesToWheels = 0;
  DriveSubsystem driveSubsystem;
  int offWheels = 0;
  Logger logger;
  int minTagsNeeded = 2;

  // Deadeye<TargetListTargetData> cam = new Deadeye<TargetListTargetData>("A0",
  // TargetListTargetData.class, NetworkTableInstance.getDefault(), null);

  private Matrix<N3, N1> adaptiveVisionMatrix;

  // Constructor
  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    logger = LoggerFactory.getLogger(this.getClass());

    cams = new WallEyeCam[VisionConstants.kNumCams];
    adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();

    // Initialize all walleyecams
    for (int i = 0; i < VisionConstants.kNumCams; ++i) {
      cams[i] = new WallEyeCam(names[i], camIndex[i], -1);
    }
  }

  // Getter/Setter Methods
  public void setVisionUpdates(boolean visionUpdates) {
    this.visionUpdates = visionUpdates;
  }

  public void setMinTagsNeeded(int num) {
    minTagsNeeded = num;
  }

  public boolean isVisionUpdatingDrive() {
    return visionUpdates;
  }

  public boolean isCameraConnected(int index) {
    return cams[index].isCameraConnected();
  }

  public VisionStates getState() {
    return curState;
  }

  public void setState(VisionStates state) {
    logger.info("{} -> {}", curState, state);
    curState = state;
  }

  // Helper Methods

  private double getSeconds() {
    return RobotController.getFPGATime() / 1000000.0;
  }

  // FIXME NEED DRIVE ODOMETRY
  private boolean isPoseValidWithWheels(WallEyeResult test, Translation3d pose) {
    if (isPoseValidWithoutWheels(test, pose)) {

      // Get Speed and current position
      ChassisSpeeds speed = driveSubsystem.getFieldRelSpeed();
      Pose2d curPose = driveSubsystem.getPoseMeters();

      // Find the x and y difference between cam and wheels
      Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));
      double magnitudeVel =
          Math.sqrt(Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2));
      double magnitudeDisp = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

      // Test to see if the displacement falls beneath a line based on velocity
      return test.getNumTags() > minTagsNeeded
          && (magnitudeDisp
              < ((magnitudeVel * VisionConstants.kLinearCoeffOnVelFilter)
                  + VisionConstants.kOffsetOnVelFilter
                  + Math.pow((magnitudeVel * VisionConstants.kSquaredCoeffOnVelFilter), 2)));
    }
    return false;
  }

  private boolean isPoseValidWithoutWheels(WallEyeResult test, Translation3d location) {
    return test.getNumTags() > minTagsNeeded
        && (test.getNumTags() >= 2 || test.getAmbiguity() <= VisionConstants.kMaxAmbig)
        && (location.getX() <= DriveConstants.kFieldMaxX)
        && (location.getY() <= DriveConstants.kFieldMaxY);
  }

  // Periodic
  @Override
  public void periodic() {

    org.littletonrobotics.junction.Logger.recordOutput("VisionSubsystem/State", curState.name());

    // cam.getEnabled();

    // If enough time elapses trust vision or if enough time elapses reset the counter
    if ((getSeconds() - timeLastVision > VisionConstants.kMaxTimeNoVision)
        && (curState != VisionStates.TRUSTVISION)) {
      //   logger.info("{} -> TRUSTVISION");
      curState = VisionStates.TRUSTVISION;
      adaptiveVisionMatrix.set(0, 0, 0.01);
      adaptiveVisionMatrix.set(1, 0, 0.01);
      updatesToWheels = 0;
    }

    // If enough time elapses between camera updates - reset count of updates to 0
    if ((getSeconds() - timeLastVision > VisionConstants.kMaxTimeNoVision)
        && updatesToWheels >= 1) {
      logger.info("Reset # of Vision updates to 0");
      updatesToWheels = 0;
    }

    // If the counter gets high enough trust wheels
    if ((updatesToWheels >= VisionConstants.kResultsForWheels)
        && curState != VisionStates.TRUSTWHEELS) {
      //   logger.info("{} -> TRUSTWHEELS", curState);
      updatesToWheels = 0;
      offWheels = 0;
      curState = VisionStates.TRUSTWHEELS;
      adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
    }

    // Clear out old results
    validResults.clear();

    // Go through each camera and store result and array idx in pair
    for (int i = 0; i < VisionConstants.kNumCams; ++i) {
      if (cams[i].hasNewUpdate()) {
        timeLastVision = getSeconds();
        validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));
      }
    }

    // Tightens std devs if time elapses
    if (getSeconds() - timeLastVision >= VisionConstants.kTimeToDecayDev
        && curState == VisionStates.TRUSTWHEELS) {

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

      // Take out data from pair
      WallEyeResult result = res.getFirst();
      int idx = res.getSecond();

      // Get center of Robot pose
      Pose3d cameraPose = result.getCameraPose();
      Translation3d centerPose =
          cameraPose
              .getTranslation()
              .minus(offsets[idx].rotateBy(cameraPose.getRotation().minus(rotsOff[idx])));

      Pose3d logPose = new Pose3d(centerPose, cameraPose.getRotation().minus(rotsOff[idx]));
      //   logger.info(
      //       "{}, {}, {}",
      //       RobotController.getFPGATime(),
      //       result.getTimeStamp(),
      //       (RobotController.getFPGATime() - result.getTimeStamp()) / 1000000.0);

      // If updating with vision go into state machine to update
      if (visionUpdates) {
        switch (curState) {

            // Uses wheels to act as a filter for the cameras
          case TRUSTWHEELS:
            if (isPoseValidWithWheels(result, centerPose)) {
              String outputAccept = "VisionSubsystem/AcceptedCam" + names[idx] + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(outputAccept, logPose.toPose2d());

              driveSubsystem.addVisionMeasurement(
                  new Pose2d(centerPose.toTranslation2d(), new Rotation2d()),
                  result.getTimeStamp() / 1000000.0,
                  adaptiveVisionMatrix);

              if (offWheels > 0) offWheels--;

            } else {

              String output = "VisionSubsystem/NotAcceptedCam" + names[idx] + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(output, logPose.toPose2d());
              offWheels++;
              if (offWheels >= VisionConstants.kMaxTimesOffWheels) {
                logger.info("{} -> TRUSTVISION", curState);
                curState = VisionStates.TRUSTVISION;
              }
            }
            break;

            // Purely trust vision
          case TRUSTVISION:
            adaptiveVisionMatrix.set(0, 0, 0.01);
            adaptiveVisionMatrix.set(1, 0, 0.01);
            if (isPoseValidWithoutWheels(result, centerPose)) {
              String outputAccept = "VisionSubsystem/AcceptedCam" + names[idx] + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(outputAccept, logPose.toPose2d());
              updatesToWheels++;

              driveSubsystem.addVisionMeasurement(
                  new Pose2d(centerPose.toTranslation2d(), new Rotation2d()),
                  result.getTimeStamp() / 1000000.0,
                  adaptiveVisionMatrix);

            } else {

              String output = "VisionSubsystem/NotAcceptedCam" + names[idx] + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(output, logPose.toPose2d());
            }

            break;
        }
      }
    }
  }

  // Grapher
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> curState.ordinal()),
        new Measure("OffWheels", () -> offWheels),
        new Measure("Updates To Wheels", () -> updatesToWheels),
        new Measure("Adaptive Vision Matrix", () -> adaptiveVisionMatrix.get(0, 0)));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  // State Enum
  public enum VisionStates {
    TRUSTWHEELS,
    TRUSTVISION
  }
}
