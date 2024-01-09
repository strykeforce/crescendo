package frc.robot.subsystems.vision;

import java.util.ArrayList;

import WallEye.WallEyeCam;
import WallEye.WallEyeResult;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    //Private Variables
    WallEyeCam[] cams;
    Translation3d[] offsets;
    String[] names = {"1", "2", "3", "4"};
    int[] camIndex = {1,2,3,4};
    ArrayList<Pair<WallEyeResult,Integer>> validResults = new ArrayList<>();
    VisionStates curState = VisionStates.TRUSTWHEELS;
    boolean visionUpdates = true;

    //Constructor
    public VisionSubsystem() {
        cams = new WallEyeCam[VisionConstants.kNumCams]; 
        for (int i = 0; i < VisionConstants.kNumCams; ++i) {
            cams[i] = new WallEyeCam(names[i], camIndex[i], -1);
        }  

    }

    //Getter/Setter Methods
    public void setVisionUpdates(boolean visionUpdates) {
        this.visionUpdates = visionUpdates;
    }

    public boolean isVisionUpdatingDrive() {
        return visionUpdates;
    }

    public boolean isCameraConnected(int index) {
        return true;
    }

    //Helper Methods


    //Periodic
    @Override
    public void periodic() {
        validResults.clear();
        for (int i = 0; i < VisionConstants.kNumCams; ++i) {
            if(cams[i].hasNewUpdate()) {
                validResults.add(new Pair<WallEyeResult,Integer>(cams[i].getResults(), i));
            }
        }
        if (visionUpdates)
        {
        switch (curState) {
            case TRUSTVISION:
            
            break;
            case TRUSTWHEELS:
            
            break;
        }
    }
    }

    //Grapher


    //State Enum
    public enum VisionStates {
        TRUSTWHEELS,
        TRUSTVISION
    }
}
