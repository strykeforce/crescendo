package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LedConstants {
  public static final int kRightLedPort = 4;
  public static final int kRightLedLength = 14;
  public static final int kLeftLedPort = 5;
  public static final int kLeftLedLength = 14;
  public static final Color kBlue = new Color(42, 45, 232);
  public static final Color kGreen = new Color(0, 255, 0);
  public static final Color kYellowish = new Color(166, 255, 0);
  public static final int kBlinkOffCount = 15;
  public static final int kBlinkOnCount = 30; // > kBlinkOffCount
}
