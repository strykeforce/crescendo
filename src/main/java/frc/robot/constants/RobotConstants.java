package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
  public static final double kRedSpeakerXPos = 16.540988;
  public static final double kRedSpeakerYPos = 5.547868;
  public static final double kBlueSpeakerXPos = 0;
  public static final double kBlueSpeakerYPos = 5.547868;
  public static final Translation2d kRedSpeakerPos =
      new Translation2d(kRedSpeakerXPos, kRedSpeakerYPos);
  public static final Translation2d kBlueSpeakerPos =
      new Translation2d(kBlueSpeakerXPos, kBlueSpeakerYPos);

  public static final Translation2d kRedFeedTargetPos =
      new Translation2d(kRedSpeakerXPos - 1, kRedSpeakerYPos + 1);
  public static final Translation2d kBlueFeedTargetPos =
      new Translation2d(kBlueSpeakerXPos + 1, kBlueSpeakerYPos + 1);

  public static final double kDegreeShootOffset = Units.degreesToRadians(-3.5); // -3.5

  // Robot Sizes
  public static final double kShooterOffset = 0.2; // meters
  public static final Rotation2d kShooterHeading = Rotation2d.fromDegrees(180);

  // Constants Different between Comp and Proto
  public static double kWheelDiameterInches = 3.0;
  public static double kElbowZero = 0.11206;
  public static double kElbowRecoveryZero = 0.11206;
  public static double kElbowZeroPos = 34.0;
  public static double kWristZero = 2322.0;

  // Climb Servos
  public static double kLeftTrapBarExtend = 0.0;
  public static double kLeftTrapBarRetract = 0.0;
  public static double kRightTrapBarExtend = 0.0;
  public static double kRightTrapBarRetract = 0.0;
  public static double kLeftRatchetOn = 0.0;
  public static double kLeftRatchetOff = 0.0;
  public static double kRightRatchetOn = 0.0;
  public static double kRightRatchetOff = 0.0;
  public static double kElbowSetpointOffset = 0.0;

  public RobotConstants() {
    logger.info("SN: {}, isCompBot: {}", RobotController.getSerialNumber(), isCompBot);
    if (isCompBot) {
      logger.info("Using Comp Robot Constants");
      kWheelDiameterInches = CompConstants.kWheelDiameterInches;
      kElbowZero = CompConstants.kElbowZero;
      kElbowRecoveryZero = CompConstants.kElbowRecoveryZero;
      kElbowZeroPos = CompConstants.kElbowZeroPos;
      kWristZero = CompConstants.kWristZero;
      kLeftTrapBarExtend = CompConstants.kLeftTrapBarExtend;
      kRightTrapBarExtend = CompConstants.kRightTrapBarExtend;
      kLeftTrapBarRetract = CompConstants.kLeftTrapBarRetract;
      kRightTrapBarRetract = CompConstants.kRightTrapBarRetract;
      kLeftRatchetOff = CompConstants.kLeftRatchetOff;
      kRightRatchetOff = CompConstants.kRightRatchetOff;
      kLeftRatchetOn = CompConstants.kLeftRatchetOn;
      kRightRatchetOn = CompConstants.kRightRatchetOn;
      kElbowSetpointOffset = CompConstants.kElbowSetpointOffset;
    } else {
      logger.info("Using Proto Robot Constants");
      kWheelDiameterInches = ProtoConstants.kWheelDiameterInches;
      kElbowZero = ProtoConstants.kElbowZero;
      kElbowRecoveryZero = ProtoConstants.kElbowRecoveryZero;
      kElbowZeroPos = ProtoConstants.kElbowZeroPos;
      kWristZero = ProtoConstants.kWristZero;
      kLeftTrapBarExtend = ProtoConstants.kLeftTrapBarExtend;
      kRightTrapBarExtend = ProtoConstants.kRightTrapBarExtend;
      kLeftTrapBarRetract = ProtoConstants.kLeftTrapBarRetract;
      kRightTrapBarRetract = ProtoConstants.kRightTrapBarRetract;
      kLeftRatchetOff = ProtoConstants.kLeftRatchetOff;
      kRightRatchetOff = ProtoConstants.kRightRatchetOff;
      kLeftRatchetOn = ProtoConstants.kLeftRatchetOn;
      kRightRatchetOn = ProtoConstants.kRightRatchetOn;
      kElbowSetpointOffset = ProtoConstants.kElbowSetpointOffset;
    }
  }

  public static class CompConstants {
    // Drive
    public static final double kWheelDiameterInches = 3.0 * 502.0 / 500.0;

    // Elbow
    public static final double kElbowZero = 0.01465; // 0.0105
    public static final double kElbowRecoveryZero = -0.2656;
    public static final double kElbowZeroPos = 30.45;
    public static final double kElbowSetpointOffset = 0.00378; // 0.00488

    // Wrist
    public static final double kWristZero = 2243.0; // 2223

    // Climb
    public static final double kLeftTrapBarExtend = 0.5;
    public static final double kLeftTrapBarRetract = 0.0;
    public static final double kRightTrapBarExtend = 0.5;
    public static final double kRightTrapBarRetract = 1.0;

    public static final double kLeftRatchetOn = 0.0;
    public static final double kLeftRatchetOff = 1.0;
    public static final double kRightRatchetOn = 0.0;
    public static final double kRightRatchetOff = 1.0;
  }

  public static class ProtoConstants {
    // Drive
    public static final double kWheelDiameterInches = 3.0 * 503.5 / 500.0;

    // Elbow
    public static final double kElbowZero = 0.23291; // -0.11816
    public static final double kElbowRecoveryZero = 0.23291;
    public static final double kElbowZeroPos = 34.0;
    public static final double kElbowSetpointOffset = 0.0;

    // Wrist
    public static final double kWristZero = 3293.0; // 3310

    // Climb
    public static final double kLeftTrapBarExtend = 0.0;
    public static final double kLeftTrapBarRetract = 0.0;
    public static final double kRightTrapBarExtend = 0.0;
    public static final double kRightTrapBarRetract = 0.0;
    public static final double kLeftRatchetOn = 0.0;
    public static final double kLeftRatchetOff = 0.0;
    public static final double kRightRatchetOn = 0.0;
    public static final double kRightRatchetOff = 0.0;
  }
}
