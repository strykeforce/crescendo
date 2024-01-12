package frc.robot.subsystems.vision;

import WallEye.WallEyeCam;
import WallEye.WallEyeResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class VisionSubsystem extends SubsystemBase {

  // Private Variables
  WallEyeCam[] cams;
  Translation3d[] offsets = {
    new Translation3d(0, 0, 0),
    new Translation3d(0, 0, 0),
    new Translation3d(0, 0, 0),
    new Translation3d(0, 0, 0)
  };
  String[] names = {"1", "2", "3", "4"};
  int[] camIndex = {1, 2, 3, 4};
  ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>();
  VisionStates curState = VisionStates.TRUSTWHEELS;
  boolean visionUpdates = true;
  double timeLastVision = 0;
  int updatesToWheels = 0;

  public Matrix<N3, N1> adaptiveVisionMatrix;

  // Constructor
  public VisionSubsystem() {
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
    curState = state;
  }

  // Helper Methods

  private double getSeconds() {
    return RobotController.getFPGATime() / 1000000.0;
  }

  // FIXME NEED DRIVE ODOMETRY
  private boolean isPoseValidWithWheels(WallEyeResult test, Translation3d pose) {
    if (isPoseValidWithoutWheels(test)) {
      // Check with drive odometry and cross reference the two BUT thats for later
      return true;
    }
    return false;
  }

  private boolean isPoseValidWithoutWheels(WallEyeResult test) {
    return test.getNumTags() >= 2 || test.getAmbiguity() <= VisionConstants.kMaxAmbig;
  }

  // Periodic
  @Override
  public void periodic() {

    // If enough time elapses trust vision or if enough time elapses reset the counter
    if (getSeconds() - timeLastVision > VisionConstants.kMaxTimeNoVision) {
      curState = VisionStates.TRUSTVISION;
      updatesToWheels = 0;
    }

    // If the counter gets high enough trust wheels
    if (updatesToWheels >= VisionConstants.kResultsForWheels) {
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
      System.out.print(result.getCameraPose());

      // Get center of Robot pose
      Pose3d cameraPose = result.getCameraPose();
      Translation3d centerPose =
          cameraPose.getTranslation().plus(offsets[idx].rotateBy(cameraPose.getRotation()));

      // If updating with vision go into state machine to update
      if (visionUpdates) {
        switch (curState) {

            // Uses wheels to act as a filter for the cameras
          case TRUSTWHEELS:
            if (isPoseValidWithWheels(result, centerPose)) {
              // Feed into odometry

            }
            break;

            // Purely trust vision
          case TRUSTVISION:
            if (isPoseValidWithoutWheels(result)) {
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
