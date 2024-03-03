package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.DefaultAutoCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.ArrayList;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch extends MeasurableSubsystem {
  public Logger logger = LoggerFactory.getLogger(AutoSwitch.class);

  private RobotStateSubsystem robotStateSubsystem;
  private SuperStructure superStructure;
  private MagazineSubsystem magazineSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private DriveSubsystem driveSubsystem;
  private ClimbSubsystem climbSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private WristSubsystem wristSubsystem;
  private ShooterSubsystem shooterSubsystem;

  public boolean useVirtualSwitch;
  private static SendableChooser<Integer> sendableChooser = new SendableChooser<>();
  private AutoCommandInterface defaultCommand;
  private AutoCommandInterface autoCommand;
  private final AutonSwitch autoSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int curAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;

  public AutoSwitch(
      RobotStateSubsystem robotStateSubsystem,
      SuperStructure superStructure,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      DriveSubsystem driveSubsystem,
      ClimbSubsystem climbSubsystem,
      ElbowSubsystem elbowSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.superStructure = superStructure;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    for (int i = RobotConstants.kMinAutoSwitchID; i <= RobotConstants.kMaxAutoSwitchID; i++) {
      switchInputs.add(new DigitalInput(i));
    }
    autoSwitch = new AutonSwitch(switchInputs);

    configSendableChooser();

    // FIXME
    defaultCommand =
        new DefaultAutoCommand(
            robotStateSubsystem,
            driveSubsystem,
            superStructure,
            magazineSubsystem,
            elbowSubsystem,
            "5mTestPath");
  }

  public void checkSwitch() {
    if (hasSwitchChanged()) {
      logger.info("Initializing Auto Switch Position: {}", String.format("%02X", curAutoSwitchPos));
      autoCommand = getAutoCommand(curAutoSwitchPos);
      if (!autoCommand.hasGenerated()) autoCommand.generateTrajectory();
    }
  }

  public void resetSwitchPos() {
    if (curAutoSwitchPos == -1) {
      logger.info("Reset Auto Switch");
    }
    curAutoSwitchPos = -1;
  }

  public AutoCommandInterface getAutoCommand() {
    if (autoCommand == null) {
      return defaultCommand;
    } else return this.autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = useVirtualSwitch ? sendableChooser.getSelected() : autoSwitch.position();

    if (switchPos != newAutoSwitchPos) {
      autoSwitchStableCounts = 0;
      newAutoSwitchPos = switchPos;
    } else autoSwitchStableCounts++;

    if (autoSwitchStableCounts > AutonConstants.kSwitchStableCounts
        && curAutoSwitchPos != newAutoSwitchPos) {
      changed = true;
      curAutoSwitchPos = newAutoSwitchPos;
    }

    return changed;
  }

  private AutoCommandInterface getAutoCommand(int switchPos) {
    switch (switchPos) {
      case 0x00:

      case 0x20:

      case 0x21:

      default:
        String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DefaultAutoCommand(
            robotStateSubsystem,
            driveSubsystem,
            superStructure,
            magazineSubsystem,
            elbowSubsystem,
            "5mTestPath");
    }
  }

  private void configSendableChooser() {
    sendableChooser.addOption("00 Source 3 piece", 0x00);
    sendableChooser.setDefaultOption("20 Amp 4 piece interfere", 0x01);
    sendableChooser.setDefaultOption("21 Amp 4 piece avoid", 0x21);
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

  public String getSwitchPos() {
    return Integer.toHexString(curAutoSwitchPos);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("usingVirtualSwitch", () -> this.useVirtualSwitch ? 1.0 : 0.0));
  }
}