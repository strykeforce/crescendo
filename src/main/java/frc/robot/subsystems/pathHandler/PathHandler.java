package frc.robot.subsystems.pathHandler;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;

import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import frc.robot.commands.DriveAutonCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.vision.DeadEyeSubsystem;

public class PathHandler extends MeasurableSubsystem{
    
    PathStates curState = PathStates.FETCH;
    DeadEyeSubsystem deadeye;
    ArrayList<Integer> noteOrder; 
    DriveAutonCommand nextPath;

    RobotStateSubsystem robotStateSubsystem;


    public PathHandler(DeadEyeSubsystem deadeye, RobotStateSubsystem robotStateSubsystem) {
        this.deadeye = deadeye;
        this.robotStateSubsystem = robotStateSubsystem;
    }

    public PathStates getState() {
        return curState;
    }

    public void setState(PathStates state) {
        curState = state;
    }

    public void setPreference(ArrayList<Integer> list) {
        noteOrder = list;
    }

    public DriveAutonCommand getNextPath() {
        return nextPath;
    }

    @Override
    public void periodic() {
        switch (curState) {
            case SHOOT:
                
                break;
            case FETCH:

                break;
        }
    }

    @Override
    public Set<Measure> getMeasures() {
        // TODO Auto-generated method stub
        return null;
    }

    public enum PathStates {
        SHOOT, FETCH
    }
}
