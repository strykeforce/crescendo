package frc.robot.subsystems.vision;

import WallEye.WallEyeCam;
import WallEye.WallEyeResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VisionSubsystem extends SubsystemBase {

  // Private Variables
  WallEyeCam[] cams;
  Translation3d[] offsets = {
    new Translation3d(0, 0, 0), new Translation3d(0, 0, 0)
  }; // FIXME: put in VisionConstants
  Rotation3d[] rotsOff = {
    new Rotation3d(0, 0, 0), new Rotation3d(0, 0, 0)
  }; // FIXME: put in VisionConstants
  String[] names = {"1", "2"};
  int[] camIndex = {1, 2};
  ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>(); // <Result, Cam #>
  VisionStates curState = VisionStates.TRUSTWHEELS;
  boolean visionUpdates = true;
  double timeLastVision = 0;
  int updatesToWheels = 0;
  DriveSubsystem driveSubsystem;
  int offWheels = 0;
  Logger logger;

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
    if (isPoseValidWithoutWheels(test)) {

      // Get Speed and current position
      ChassisSpeeds speed = driveSubsystem.getFieldRelSpeed();
      Pose2d curPose = driveSubsystem.getPoseMeters();

      // Find the x and y difference between cam and wheels
      Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));
      double magnitudeVel =
          Math.sqrt(Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2));
      double magnitudeDisp = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

      // Test to see if the displacement falls beneath a line based on velocity
      return (magnitudeDisp
          < ((magnitudeVel * VisionConstants.kLinearCoeffOnVelFilter)
              + VisionConstants.kOffsetOnVelFilter
              + Math.pow((magnitudeVel * VisionConstants.kSquaredCoeffOnVelFilter), 2)));
    }
    return false;
  }

  private boolean isPoseValidWithoutWheels(WallEyeResult test) {
    return test.getNumTags() >= 2 || test.getAmbiguity() <= VisionConstants.kMaxAmbig;
  }

  // Periodic
  @Override
  public void periodic() {

    // cam.getEnabled();

    // If enough time elapses trust vision or if enough time elapses reset the counter
    if ((getSeconds() - timeLastVision > VisionConstants.kMaxTimeNoVision)
        && (curState != VisionStates.TRUSTVISION)) {
      logger.info("{} -> TRUSTVISION");
      curState = VisionStates.TRUSTVISION;
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
      logger.info("{} -> TRUSTWHEELS", curState);
      updatesToWheels = 0;
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
    if (getSeconds() - timeLastVision >= VisionConstants.kTimeToDecayDev) {

      // Take x and y weights and linearly decrease them
      for (int i = 0; i < 2; ++i) {
        double estimatedWeight =
            VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                - VisionConstants.kStdDevDecayCoeff
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
              .plus(offsets[idx].rotateBy(cameraPose.getRotation().minus(rotsOff[idx])));

      String output = "VisionSubsystem/Cam" + res.getSecond().toString() + "Pose";
      org.littletonrobotics.junction.Logger.recordOutput(output, centerPose);

      // If updating with vision go into state machine to update
      if (visionUpdates) {
        switch (curState) {

            // Uses wheels to act as a filter for the cameras
          case TRUSTWHEELS:
            if (isPoseValidWithWheels(result, centerPose)) {
              String outputAccept =
                  "VisionSubsystem/AcceptedCam" + res.getSecond().toString() + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(outputAccept, centerPose);
              // Feed into odometry
              offWheels--;

            } else {
              offWheels = offWheels < 0 ? 1 : offWheels++;
              if (offWheels >= VisionConstants.kMaxTimesOffWheels) {
                logger.info("{} -> TRUSTVISION", curState);
                curState = VisionStates.TRUSTVISION;
              }
            }
            break;

            // Purely trust vision
          case TRUSTVISION:
            if (isPoseValidWithoutWheels(result)) {
              String outputAccept =
                  "VisionSubsystem/AcceptedCam" + res.getSecond().toString() + "Pose";
              org.littletonrobotics.junction.Logger.recordOutput(outputAccept, centerPose);
              updatesToWheels++;
              // Feed into odometry
            }

            break;
        }
      }
    }
  }

  // Grapher

  // State Enum
  public enum VisionStates {
    TRUSTWHEELS,
    TRUSTVISION
  }
}
