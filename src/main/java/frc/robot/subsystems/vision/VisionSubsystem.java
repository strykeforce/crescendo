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
  Translation3d[] offsets;
  String[] names = {"1", "2", "3", "4"};
  int[] camIndex = {1, 2, 3, 4};
  ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>();
  VisionStates curState = VisionStates.TRUSTWHEELS;
  boolean visionUpdates = true;
  long timeLastVision = 0;
  int updatesToWheels = 0;

  public Matrix<N3, N1> adaptiveVisionMatrix;

  // Constructor
  public VisionSubsystem() {
    cams = new WallEyeCam[VisionConstants.kNumCams];
    adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();

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

  private boolean isPoseValidWithWheels(Translation3d temp) {
    return true;
  }

  private boolean isPoseValidWithoutWheels(Translation3d temp) {
    return true;
  }

  // Periodic
  @Override
  public void periodic() {
    if (RobotController.getFPGATime() > VisionConstants.kMaxTimeNoVision) {
      curState = VisionStates.TRUSTVISION;
      adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
      updatesToWheels = 0;
    }

    if (updatesToWheels >= VisionConstants.kResultsForWheels) {
      curState = VisionStates.TRUSTWHEELS;
    }

    validResults.clear();
    for (int i = 0; i < VisionConstants.kNumCams; ++i) {
      if (cams[i].hasNewUpdate()) {
        timeLastVision = RobotController.getFPGATime();
        validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));
      }
    }

    if (validResults.isEmpty()
        && RobotController.getFPGATime() - timeLastVision >= VisionConstants.kTimeToDecayDev) {
      for (int i = 0; i < 2; ++i) {
        double estimatedWeight =
            VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                - VisionConstants.kStdDevDecayCoeff
                    * (RobotController.getFPGARevision()
                        - timeLastVision
                        - VisionConstants.kTimeToDecayDev)
                    / 1000000;
        adaptiveVisionMatrix.set(
            i,
            0,
            estimatedWeight > VisionConstants.kMinStdDev
                ? estimatedWeight
                : VisionConstants.kMinStdDev);
      }
    }

    for (Pair<WallEyeResult, Integer> res : validResults) {
      Pose3d cameraPose = res.getFirst().getCameraPose();
      Translation3d centerPose =
          cameraPose
              .getTranslation()
              .plus(offsets[res.getSecond()].rotateBy(cameraPose.getRotation()));

      if (visionUpdates) {
        switch (curState) {
          case TRUSTWHEELS:
            if (isPoseValidWithWheels(centerPose)) {
              // Feed into odometry

            }
            break;
          case TRUSTVISION:
            if (isPoseValidWithoutWheels(centerPose)) {
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
