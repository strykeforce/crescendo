package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RobotConstants {
  private Logger logger = LoggerFactory.getLogger(this.getClass());
  public static String kProtoSerialNumber = "030dbdd8";
  public static boolean isCompBot = (!RobotController.getSerialNumber().equals(kProtoSerialNumber));

  public static final int kTalonConfigTimeout = 10; // ms

  // Joysticks
  public static final double kJoystickDeadband = 0.1;

  // Roborio ID's
  public static final int kMinAutoSwitchID = 0;
  public static final int kMaxAutoSwitchID = 5;
  public static final int kEventInterlockID = 6;

  // Field Positions
  // FIXME: need to measure or get distance from CAD or something
  public static final double kSpeakerXPos = 0;
  public static final double kSpeakerYPos = 0;
  public static final Translation2d kSpeakerPos = new Translation2d(kSpeakerXPos, kSpeakerYPos);

  // Robot Sizes
  public static final double kShooterOffset = 0.2; // meters

  // Constants Different between Comp and Proto
  public static Double kWheelDiameterInches = 3.0;

  public RobotConstants() {
    if (isCompBot) {
      logger.info("Using Comp Robot Constants");
      kWheelDiameterInches = CompConstants.kWheelDiameterInches;
    } else {
      logger.info("Using Proto Robot Constants");
      kWheelDiameterInches = ProtoConstants.kWheelDiameterInches;
    }
  }

  public static class CompConstants {
    // Drive
    public static final Double kWheelDiameterInches = 3.0;
  }

  public static class ProtoConstants {
    // Drive
    public static final Double kWheelDiameterInches = 3.0;
  }
}
