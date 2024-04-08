package frc.robot.subsystems.vision;

import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Set;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DeadEyeSubsystem extends MeasurableSubsystem {

  Deadeye<TargetListTargetData> cam;
  double centerX = 0.0;
  TargetListTargetData data = new TargetListTargetData();

  public DeadEyeSubsystem() {
    cam = new Deadeye<>("W0", TargetListTargetData.class);
    centerX = cam.getCapture().width / 2.0;
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

  public double getDistanceToCamCenter() {
    if (data == null) return 0.0;

    ArrayList<Rect> validTargs = new ArrayList<Rect>();
    for (Rect targ : data.targets) {
      //   data.targetsOrderedByBottomRightX()
      if (targ.center().y <= VisionConstants.kBumperPixelLine) validTargs.add(targ);
    }
    validTargs.sort(
        Comparator.comparingInt(r -> -((Rect) r).center().y)
            .thenComparingInt(r -> Math.abs(((int) centerX - ((Rect) r).center().x))));
    if (validTargs.size() > 0) return validTargs.get(0).center().x - centerX;
    return 0.0;
  }

  private double getTargetY() {
    double lastValidTargY = 2767.0;
    for (Rect targ : data.targets) {
      lastValidTargY = targ.center().y;
    }
    return lastValidTargY;
  }

  public int getNumTargets() {
    return data.targets.size();
  }

  //   public boolean isNoteRight() {
  //     List<Rect> list = data.targets;

  //     for (Rect rect : list) {
  //       if (rect.topLeft.x >= border2) return true;
  //     }
  //     return false;
  //   }

  //   public boolean isNoteMid() {
  //     List<Rect> list = data.targets;

  //     for (Rect rect : list) {
  //       if (rect.topLeft.x >= border1 && rect.bottomRight.x <= border2) return true;
  //     }
  //     return false;
  //   }

  //   public boolean isNoteLeft() {
  //     List<Rect> list = data.targets;

  //     for (Rect rect : list) {
  //       if (rect.bottomRight.x <= border1) return true;
  //     }
  //     return false;
  //   }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput(
        "Deadeye/TargetDistanceToCenter", getDistanceToCamCenter());
    TargetListTargetData newData = cam.getTargetData();
    if (newData != null) {
      data = newData;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Is deadeye enabled", () -> cam.getEnabled() ? 1 : 0),
        new Measure("Y Pixel", () -> getTargetY()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
