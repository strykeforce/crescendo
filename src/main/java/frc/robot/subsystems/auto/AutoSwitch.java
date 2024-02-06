package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AutoSwitch extends MeasurableSubsystem {
  public boolean useVirtualSwitch;
  private static SendableChooser<Integer> sendableChooser = new SendableChooser<>();
  public Logger logger = LoggerFactory.getLogger(AutoSwitch.class);

  public AutoSwitch() {
    sendableChooser.addOption("TEST", 0x00);
    sendableChooser.setDefaultOption("TEST", 0x01);
    sendableChooser.setDefaultOption("TEST", 0x03);
    SmartDashboard.putData("Auto Mode", sendableChooser);
  }

  public void toggleVirtualSwitch() {
    logger.info("toggledSwitch:function");
    if (useVirtualSwitch) {
      useVirtualSwitch = false;
    } else {
      useVirtualSwitch = true;
    }
    // useVirtualSwitch = useVirtualSwitch ? false : true;
  }

  public SendableChooser<Integer> getSendableChooser() {
    return sendableChooser;
  }

  public boolean isUseVirtualSwitch() {
    return useVirtualSwitch;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("usingVirtualSwitch", () -> this.useVirtualSwitch ? 1.0 : 0.0));
  }
}
