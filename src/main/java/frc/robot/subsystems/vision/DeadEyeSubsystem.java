package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Set;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DeadEyeSubsystem extends MeasurableSubsystem {

  Deadeye<TargetListTargetData> cam;

  TargetListTargetData data = new TargetListTargetData();

  int borderLengthX = 0;
  int border1 = 0;
  int border2 = 0;

  public DeadEyeSubsystem() {
    cam = new Deadeye<>("W0", TargetListTargetData.class);
    borderLengthX = cam.getCapture().width / 3;
    border1 = borderLengthX;
    border2 = borderLengthX + border1;
  }

  public boolean isCamEnabled() {
    return cam.getEnabled();
  }

  public void setCamEnabled(boolean val) {
    cam.setEnabled(val);
  }

  public TargetListTargetData getTargetListData() {
    return data;
  }

  public boolean isNoteRight() {
    List<Rect> list = data.targets;

    for (Rect rect : list) {
      if (rect.topLeft.x >= border2) return true;
    }
    return false;
  }

  public boolean isNoteMid() {
    List<Rect> list = data.targets;

    for (Rect rect : list) {
      if (rect.topLeft.x >= border1 && rect.bottomRight.x <= border2) return true;
    }
    return false;
  }

  public boolean isNoteLeft() {
    List<Rect> list = data.targets;

    for (Rect rect : list) {
      if (rect.bottomRight.x <= border1) return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    data = cam.getTargetData();

    System.err.println(isNoteLeft() + " " + isNoteMid() + " " + isNoteRight());
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    // TODO Auto-generated method stub
    super.registerWith(telemetryService);
  }
}