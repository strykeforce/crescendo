package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RobotConstants {
  private Logger logger = LoggerFactory.getLogger(this.getClass());
  public static String kProtoSerialNumber = "032243DF";
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
  public static final double kRedSpeakerXPos = 0;
  public static final double kRedSpeakerYPos = 5.547868;
  public static final double kBlueSpeakerXPos = 0;
  public static final double kBlueSpeakerYPos = 5.547868;
  public static final Translation2d kRedSpeakerPos =
      new Translation2d(kRedSpeakerXPos, kRedSpeakerYPos);
  public static final Translation2d kBlueSpeakerPos =
      new Translation2d(kBlueSpeakerXPos, kBlueSpeakerYPos);

  // Robot Sizes
  public static final double kShooterOffset = 0.2; // meters
  public static final Rotation2d kShooterHeading = Rotation2d.fromDegrees(180);

  // Constants Different between Comp and Proto
  public static double kWheelDiameterInches = 3.0;
  public static double kElbowZero = 0.11206;
  public static double kWristZero = 1132.0;

  // Climb Servos
  public static double kLeftTrapBarExtend = 0.0;
  public static double kLeftTrapBarRetract = 0.0;
  public static double kRightTrapBarExtend = 0.0;
  public static double kRightTrapBarRetract = 0.0;
  public static double kLeftRatchetOn = 0.0;
  public static double kLeftRatchetOff = 0.0;
  public static double kRightRatchetOn = 0.0;
  public static double kRightRatchetOff = 0.0;

  public RobotConstants() {
    logger.info("SN: {}, isCompBot: {}", RobotController.getSerialNumber(), isCompBot);
    if (isCompBot) {
      logger.info("Using Comp Robot Constants");
      kWheelDiameterInches = CompConstants.kWheelDiameterInches;
      kElbowZero = CompConstants.kElbowZero;
      kWristZero = CompConstants.kWristZero;
      kLeftTrapBarExtend = CompConstants.kLeftTrapBarExtend;
      kRightTrapBarExtend = CompConstants.kRightTrapBarExtend;
      kLeftTrapBarRetract = CompConstants.kLeftTrapBarRetract;
      kRightTrapBarRetract = CompConstants.kRightTrapBarRetract;
      kLeftRatchetOff = CompConstants.kLeftRatchetOff;
      kRightRatchetOff = CompConstants.kRightRatchetOff;
      kLeftRatchetOn = CompConstants.kLeftRatchetOn;
      kRightRatchetOn = CompConstants.kRightRatchetOn;
    } else {
      logger.info("Using Proto Robot Constants");
      kWheelDiameterInches = ProtoConstants.kWheelDiameterInches;
      kElbowZero = ProtoConstants.kElbowZero;
      kWristZero = ProtoConstants.kWristZero;
      kLeftTrapBarExtend = ProtoConstants.kLeftTrapBarExtend;
      kRightTrapBarExtend = ProtoConstants.kRightTrapBarExtend;
      kLeftTrapBarRetract = ProtoConstants.kLeftTrapBarRetract;
      kRightTrapBarRetract = ProtoConstants.kRightTrapBarRetract;
      kLeftRatchetOff = ProtoConstants.kLeftRatchetOff;
      kRightRatchetOff = ProtoConstants.kRightRatchetOff;
      kLeftRatchetOn = ProtoConstants.kLeftRatchetOn;
      kRightRatchetOn = ProtoConstants.kRightRatchetOn;
    }
  }

  public static class CompConstants {
    // Drive
    public static final Double kWheelDiameterInches = 3.0;

    // Elbow
    public static final Double kElbowZero = 0.01465; // 0.0105

    // Wrist
    public static final Double kWristZero = 2603.0;

    // Climb
    public static final Double kLeftTrapBarExtend = 0.8;
    public static final Double kLeftTrapBarRetract = 0.36;
    public static final Double kRightTrapBarExtend = 0.18;
    public static final Double kRightTrapBarRetract = 0.6;

    public static final Double kLeftRatchetOn = 0.2;
    public static final Double kLeftRatchetOff = 0.8;
    public static final double kRightRatchetOn = 0.4;
    public static final double kRightRatchetOff = 1.0;
  }

  public static class ProtoConstants {
    // Drive
    public static final Double kWheelDiameterInches = 3.0 * 503.5 / 500.0;

    // Elbow
    public static final Double kElbowZero = 0.23291; // -0.11816

    // Wrist
    public static final Double kWristZero = 3293.0; // 3310

    // Climb
    public static final Double kLeftTrapBarExtend = 0.0;
    public static final Double kLeftTrapBarRetract = 0.0;
    public static final Double kRightTrapBarExtend = 0.0;
    public static final Double kRightTrapBarRetract = 0.0;
    public static final Double kLeftRatchetOn = 0.0;
    public static final Double kLeftRatchetOff = 0.0;
    public static final double kRightRatchetOn = 0.0;
    public static final double kRightRatchetOff = 0.0;
  }
}
