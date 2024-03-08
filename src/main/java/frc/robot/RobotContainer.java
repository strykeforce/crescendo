// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.ToggleVirtualSwitchCommand;
import frc.robot.commands.auton.AmpInitial_WingNotes_ACommand;
import frc.robot.commands.auton.AmpInitial_WingNotes_BCommand;
import frc.robot.commands.auton.NonAmpAutoCommand;
import frc.robot.commands.auton.NonAmpInit_TravelNotesCommand;
import frc.robot.commands.auton.NonAmpInitial_Note3Command;
import frc.robot.commands.auton.ToggleIsAutoCommand;
import frc.robot.commands.climb.ForkOpenLoopCommand;
import frc.robot.commands.climb.HoldClimbCommand;
import frc.robot.commands.climb.IncrementRequestPrepClimbCommand;
import frc.robot.commands.climb.JogClimbClosedLoopCommand;
import frc.robot.commands.climb.ToggleRatchetCommand;
import frc.robot.commands.climb.ToggleTrapBarPosCommand;
import frc.robot.commands.climb.ZeroClimbCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.ToggleVisionUpdatesCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.elbow.ClosedLoopElbowCommand;
import frc.robot.commands.elbow.HoldElbowCommand;
import frc.robot.commands.elbow.JogElbowClosedLoopCommand;
import frc.robot.commands.elbow.OpenLoopElbowCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.elbow.ZeroRecoveryElbowCommand;
import frc.robot.commands.magazine.OpenLoopMagazineCommand;
import frc.robot.commands.robotState.AmpCommand;
import frc.robot.commands.robotState.ClimbCommand;
import frc.robot.commands.robotState.ClimbTrapDecendCommand;
import frc.robot.commands.robotState.DecendCommand;
import frc.robot.commands.robotState.FullTrapClimbCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.commands.robotState.PodiumCommand;
import frc.robot.commands.robotState.PostClimbStowCommand;
import frc.robot.commands.robotState.PrepClimbCommand;
import frc.robot.commands.robotState.ReleaseNoteCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.ToggleDefenseCommand;
import frc.robot.commands.robotState.TunedShotCommand;
import frc.robot.commands.robotState.TuningOffCommand;
import frc.robot.commands.robotState.TuningShootCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.commands.wrist.ClosedLoopWristCommand;
import frc.robot.commands.wrist.OpenLoopWristCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.auto.AutoSwitch;
import frc.robot.subsystems.climb.ClimbIOFX;
import frc.robot.subsystems.climb.ClimbRatchetIOServo;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ForkIOSRX;
import frc.robot.subsystems.climb.TrapBarIOServo;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elbow.ElbowIOFX;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeIOFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineIOFX;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.shooter.ShooterIOFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superStructure.SuperStructure;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristIOSRX;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.Map;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  private final RobotConstants robotConstants;
  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final SuperStructure superStructure;
  private final RobotStateSubsystem robotStateSubsystem;
  private final MagazineSubsystem magazineSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final AutoSwitch autoSwitch;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);

  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private Alliance alliance = Alliance.Blue;
  private SuppliedValueWidget<Boolean> allianceColor;
  private Boolean isEvent = true;

  private NonAmpAutoCommand nonAmpAutonPath;
  private NonAmpAutoCommand nonAmpAutoNote3;
  private NonAmpInitial_Note3Command nonAmpNote3;
  private NonAmpInit_TravelNotesCommand nonAmpTravelNotes;
  private AmpInitial_WingNotes_BCommand ampInitial_WingNotes_BCommand;
  private AmpInitial_WingNotes_ACommand ampInitial_WingNotes_ACommand;
  // private HoloContTuningCommand holoContTuningCommand;
  private DriveAutonCommand calibrateWheelSize;
  public GenericEntry lShooterSpeed;
  public GenericEntry rShooterSpeed;
  public GenericEntry magazineSpeed;
  public GenericEntry elbowPos;
  public GenericEntry duplicateShooters;
  public GenericEntry shootDelay;

  public RobotContainer() {
    robotConstants = new RobotConstants();
    driveSubsystem = new DriveSubsystem(new Swerve());
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    wristSubsystem = new WristSubsystem(new WristIOSRX());
    elbowSubsystem = new ElbowSubsystem(new ElbowIOFX());
    shooterSubsystem = new ShooterSubsystem(new ShooterIOFX());
    intakeSubsystem = new IntakeSubsystem(new IntakeIOFX());
    magazineSubsystem = new MagazineSubsystem(new MagazineIOFX());
    climbSubsystem =
        new ClimbSubsystem(
            new ClimbIOFX(), new ClimbRatchetIOServo(), new TrapBarIOServo(), new ForkIOSRX());

    intakeSubsystem.setFwdLimitSwitchSupplier(driveSubsystem.getAzimuth1FwdLimitSupplier());

    superStructure =
        new SuperStructure(wristSubsystem, elbowSubsystem, shooterSubsystem, magazineSubsystem);

    robotStateSubsystem =
        new RobotStateSubsystem(
            visionSubsystem,
            driveSubsystem,
            intakeSubsystem,
            magazineSubsystem,
            superStructure,
            climbSubsystem);

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);
    autoSwitch =
        new AutoSwitch(
            robotStateSubsystem,
            superStructure,
            magazineSubsystem,
            intakeSubsystem,
            driveSubsystem,
            climbSubsystem,
            elbowSubsystem,
            wristSubsystem,
            shooterSubsystem);

    // visionSubsystem.setVisionUpdates(false);
    nonAmpAutonPath =
        new NonAmpAutoCommand(
            driveSubsystem,
            robotStateSubsystem,
            superStructure,
            magazineSubsystem,
            intakeSubsystem,
            elbowSubsystem,
            "NonAmpInitial1_MiddleNote5",
            "MiddleNote5_NonAmpShoot2",
            "NonAmpShoot2_MiddleNote4",
            "MiddleNote4_NonAmpShoot2");
    nonAmpAutonPath.generateTrajectory();

    // nonAmpNote3 =
    //     new NonAmpInitial_Note3Command(
    //         driveSubsystem,
    //         robotStateSubsystem,
    //         superStructure,
    //         magazineSubsystem,
    //         intakeSubsystem);
    // nonAmpNote3.generateTrajectory();

    nonAmpTravelNotes =
        new NonAmpInit_TravelNotesCommand(
            driveSubsystem,
            robotStateSubsystem,
            superStructure,
            magazineSubsystem,
            intakeSubsystem);
    nonAmpTravelNotes.generateTrajectory();

    nonAmpAutoNote3 =
        new NonAmpAutoCommand(
            driveSubsystem,
            robotStateSubsystem,
            superStructure,
            magazineSubsystem,
            intakeSubsystem,
            elbowSubsystem,
            "NonAmpInitial1_MiddleNote3",
            "MiddleNote3_NonAmpShoot2",
            "NonAmpShoot2_MiddleNote4_B",
            "MiddleNote4_NonAmpShoot2_B");
    nonAmpAutoNote3.generateTrajectory();

    // holoContTuningCommand = new HoloContTuningCommand(driveSubsystem);
    // holoContTuningCommand.generateTrajectory();

    // calibrateWheelSize = new DriveAutonCommand(driveSubsystem, "5mTestPath", true, true);
    // calibrateWheelSize.generateTrajectory();

    configureDriverBindings();
    configureOperatorBindings();
    // configureClimbTestBindings();
    configureMatchDashboard();
    configureDebugDashboard();
    configurePitDashboard();
    configureTuningDashboard();
    robotStateSubsystem.setAllianceColor(Alliance.Blue);

    // configureTelemetry();
    // configurePitDashboard();
  }

  public boolean hasElbowZeroed() {
    return elbowSubsystem.hasZeroed();
  }

  public boolean hasClimbZeroed() {
    return climbSubsystem.hasClimbZeroed();
  }

  public Command getClimbZeroCommand() {
    return new ZeroClimbCommand(climbSubsystem);
  }

  public void zeroElbow() {
    elbowSubsystem.zero();
  }

  public void zeroClimb() {
    climbSubsystem.zeroAll();
  }

  public void configurePitDashboard() {
    Shuffleboard.getTab("Pit")
        .add(
            "Stow",
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem))
        .withSize(1, 1)
        .withPosition(0, 0);

    Shuffleboard.getTab("Pit")
        .add("Elbow Pos Zero ", new ClosedLoopElbowCommand(elbowSubsystem, 0.0))
        .withSize(1, 1)
        .withPosition(1, 0);
    Shuffleboard.getTab("Pit")
        .add("Zero Elbow", new ZeroElbowCommand(elbowSubsystem))
        .withSize(1, 1)
        .withPosition(2, 0);

    Shuffleboard.getTab("Pit")
        .add("Wrist Pos Zero", new ClosedLoopWristCommand(wristSubsystem, 0))
        .withSize(1, 1)
        .withPosition(3, 0);

    Shuffleboard.getTab("Pit")
        .add("Zero Climb", new ZeroClimbCommand(climbSubsystem))
        .withSize(1, 1)
        .withPosition(4, 0);

    Shuffleboard.getTab("Pit")
        .add("Toggle isAuto", new ToggleIsAutoCommand(robotStateSubsystem))
        .withSize(1, 1)
        .withPosition(0, 1);

    Shuffleboard.getTab("Pit")
        .addBoolean("isAuto", () -> robotStateSubsystem.getIsAuto())
        .withSize(1, 1)
        .withPosition(1, 1);

    Shuffleboard.getTab("Pit")
        .add("Lock Wheels Zero", new LockZeroCommand(driveSubsystem))
        .withSize(1, 1)
        .withPosition(3, 1);
    //     Shuffleboard.getTab("Pit")
    // .add("Elbow to zero", new ClosedLoopElbowCommand(elbowSubsystem, 0))
    // .withSize(1, 1)
    // .withPosition(2, 0);

    // // Climb buttons
    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Prep Climb", new PrepClimbCommand(robotStateSubsystem, climbSubsystem,
    // superStructure))
    //     .withSize(1, 1)
    //     .withPosition(0, 2);
    // Shuffleboard.getTab("Pit")
    //     .add("Climb", new ClimbCommand(robotStateSubsystem, climbSubsystem, superStructure))
    //     .withSize(1, 1)
    //     .withPosition(1, 2);

    // Shuffleboard.getTab("Pit")
    //     .add("To Trap", new TrapCommand(robotStateSubsystem, climbSubsystem, superStructure))
    //     .withSize(1, 1)
    //     .withPosition(2, 2);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Score Trap (DECEND)",
    //         new ScoreTrapCommand(
    //             robotStateSubsystem, climbSubsystem, superStructure, magazineSubsystem, true))
    //     .withSize(1, 1)
    //     .withPosition(3, 2);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Score Trap (STAY)",
    //         new ScoreTrapCommand(
    //             robotStateSubsystem, climbSubsystem, superStructure, magazineSubsystem, false))
    //     .withSize(1, 1)
    //     .withPosition(4, 2);

    // Shuffleboard.getTab("Pit")
    //     .add("Decend", new DecendCommand(robotStateSubsystem, climbSubsystem, superStructure))
    //     .withSize(1, 1)
    //     .withPosition(5, 2);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Post Climb Stow",
    //         new PostClimbStowCommand(
    //             robotStateSubsystem,
    //             superStructure,
    //             magazineSubsystem,
    //             intakeSubsystem,
    //             climbSubsystem))
    //     .withSize(1, 1)
    //     .withPosition(6, 2);

    // Shuffleboard.getTab("Pit")
    //     .add("gyro to 60", new setAngleOffsetCommand(driveSubsystem, 60))
    //     .withSize(1, 1)
    //     .withPosition(0, 1);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Set Gyro offset -50",
    //         new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-50)))
    //     .withSize(1, 1)
    //     .withPosition(3, 0);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "DistanceShoot",
    //         new DistanceShootCommand(
    //             robotStateSubsystem,
    //             superStructure,
    //             magazineSubsystem,
    //             intakeSubsystem,
    //             AutonConstants.kAI1ToSpeakerDist))
    //     .withSize(1, 1)
    //     .withPosition(4, 0);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "Full Trap Sequence",
    //         new FullTrapClimbCommand(robotStateSubsystem, climbSubsystem, superStructure))
    //     .withSize(1, 1)
    //     .withPosition(7, 2);

    // Shuffleboard.getTab("Pit")
    //     .add(
    //         "PositionShoot",
    //         new PositionShootCommand(
    //             robotStateSubsystem,
    //             superStructure,
    //             magazineSubsystem,
    //             intakeSubsystem,
    //             new Pose2d(3, AutonConstants.Setpoints.W2.getY(), Rotation2d.fromDegrees(0.0))))
    //     .withSize(1, 1)
    //     .withPosition(1, 1);
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Blue)
            .withProperties(Map.of("colorWhenFalse", "blue", "colorWhenTrue", "red"))
            .withSize(2, 2)
            .withPosition(0, 0);

    // Alliance color toggle at 2,0 position (robot.java)

    Shuffleboard.getTab("Match")
        .addBoolean("Have Note", () -> robotStateSubsystem.hasNote())
        .withSize(1, 1)
        .withPosition(3, 0);

    Shuffleboard.getTab("Match")
        .addBoolean(
            "Cams Connected",
            () -> visionSubsystem.isCameraConnected(0) && visionSubsystem.isCameraConnected(1))
        .withSize(1, 1)
        .withPosition(4, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Elbow Limit Switch On", () -> elbowSubsystem.getRevLimitSwitch())
        .withSize(1, 1)
        .withPosition(7, 1);

    Shuffleboard.getTab("Match")
        .addBoolean("Is NavX Connected", () -> driveSubsystem.isNavxWorking())
        .withSize(1, 1)
        .withPosition(5, 0);
    Shuffleboard.getTab("Match")
        .addDouble("Navx Update Number", () -> driveSubsystem.getPoseMeters().getRotation())
        .withSize(1, 1)
        .withPosition(7, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Is Traj Generated", () -> autoSwitch.getAutoCommand().hasGenerated())
        .withSize(1, 1)
        .withPosition(2, 1);
    Shuffleboard.getTab("Match")
        .addString("AutoSwitchPos", () -> autoSwitch.getSwitchPos())
        .withSize(1, 1)
        .withPosition(3, 1);
    Shuffleboard.getTab("Match")
        .add("ToggleVirtualSwitch", new ToggleVirtualSwitchCommand(autoSwitch))
        .withSize(1, 1)
        .withPosition(4, 1);
    Shuffleboard.getTab("Match")
        .addBoolean("Is VirtualSwitch Used", () -> autoSwitch.isUseVirtualSwitch())
        .withSize(1, 1)
        .withPosition(4, 2);
    Shuffleboard.getTab("Match")
        .add("VirtualAutoSwitch", autoSwitch.getSendableChooser())
        .withSize(1, 1)
        .withPosition(5, 1);

    Shuffleboard.getTab("Match")
        .add("ToggleVisionUpdates", new ToggleVisionUpdatesCommand(driveSubsystem))
        .withSize(1, 1)
        .withPosition(6, 0);

    Shuffleboard.getTab("Match")
        .addBoolean("Vision updates enabled", () -> driveSubsystem.usingVisionUpdates())
        .withSize(1, 1)
        .withPosition(6, 1);
    // Shuffleboard.getTab("Match")
    //     .add("ZeroRecoveryElbowCommand", new ZeroRecoveryElbowCommand(elbowSubsystem))
    //     .withSize(1, 1)
    //     .withPosition(7, 1);
  }

  public void configureDebugDashboard() {
    Shuffleboard.getTab("Debug")
        .add(new DecendCommand(robotStateSubsystem, climbSubsystem, superStructure))
        .withSize(1, 1)
        .withPosition(0, 0);
    Shuffleboard.getTab("Debug")
        .add(new ZeroClimbCommand(climbSubsystem))
        .withSize(1, 1)
        .withPosition(1, 0);
    Shuffleboard.getTab("Debug")
        .add(new ZeroRecoveryElbowCommand(elbowSubsystem))
        .withSize(1, 1)
        .withPosition(2, 0);
  }

  public void configureTuningDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Tuning");
    lShooterSpeed =
        tab.add("left shooter speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rShooterSpeed =
        tab.add("right shooter speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    magazineSpeed = tab.add("Magazine speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    elbowPos = tab.add("Elbow Position", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    duplicateShooters =
        tab.add("Duplicate Shooters?", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    shootDelay = tab.add("Shoot Delay", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    tab.add(
        "shoot",
        new TuningShootCommand(
            robotStateSubsystem,
            superStructure,
            magazineSubsystem,
            intakeSubsystem,
            () -> lShooterSpeed.getDouble(0.0),
            () -> rShooterSpeed.getDouble(0.0),
            () -> magazineSpeed.getDouble(0.0),
            () -> elbowPos.getDouble(0.0),
            () -> duplicateShooters.getBoolean(true),
            () -> shootDelay.getDouble(0.0)));
    tab.add(
        "stop",
        new TuningOffCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    tab.add("start", new TunedShotCommand(robotStateSubsystem, superStructure, magazineSubsystem));
  }

  public void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    wristSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);
    shooterSubsystem.registerWith(telemetryService);
    superStructure.registerWith(telemetryService);
    climbSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    magazineSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "blue"));
    robotStateSubsystem.setAllianceColor(alliance);

    autoSwitch.getAutoCommand().generateTrajectory();

    // Flips gyro angle if alliance is red team
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
    }
  }

  public Alliance getAllianceColor() {
    return alliance;
  }

  public AutoSwitch getAutoSwitch() {
    return this.autoSwitch;
  }

  public void setIsEvent(boolean isEvent) {
    this.isEvent = isEvent;
  }

  public void setIsAuto(boolean isAuto) {
    robotStateSubsystem.setIsAuto(isAuto);
  }

  private void configureOperatorBindings() {
    // Open Loop Wrist
    new Trigger((() -> xboxController.getLeftY() > RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopWristCommand(wristSubsystem, 0.2))
        .onFalse(new OpenLoopWristCommand(wristSubsystem, 0.0));
    new Trigger((() -> xboxController.getLeftY() < -RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopWristCommand(wristSubsystem, -0.2))
        .onFalse(new OpenLoopWristCommand(wristSubsystem, 0.0));

    // Open Loop Elbow
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopElbowCommand(elbowSubsystem, 0.1))
        .onFalse(new OpenLoopElbowCommand(elbowSubsystem, 0));
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopElbowCommand(elbowSubsystem, -0.1))
        .onFalse(new OpenLoopElbowCommand(elbowSubsystem, 0));

    // Climb
    new Trigger((() -> xboxController.getLeftTriggerAxis() > 0.5))
        .onTrue(new ClimbCommand(robotStateSubsystem, climbSubsystem, superStructure));
    new Trigger((() -> xboxController.getRightTriggerAxis() > 0.5))
        .onTrue(new FullTrapClimbCommand(robotStateSubsystem, climbSubsystem, superStructure));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new ClimbTrapDecendCommand(robotStateSubsystem, climbSubsystem, superStructure));
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(new PrepClimbCommand(robotStateSubsystem, climbSubsystem, superStructure))
        .onTrue(new IncrementRequestPrepClimbCommand(climbSubsystem));

    // Amp Prep
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(
            new AmpCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    // new JoystickButton(xboxController, XboxController.Button.kB.value)
    //     .onTrue(new OpenLoopMagazineCommand(magazineSubsystem, .2))
    //     .onFalse(new OpenLoopMagazineCommand(magazineSubsystem, 0));

    // Podium Prep
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(
            new PodiumCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // SubWoofer
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Defense
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new ToggleDefenseCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new PostClimbStowCommand(
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem,
                climbSubsystem));

    // // Run auton
    // new JoystickButton(xboxController, XboxController.Button.kStart.value)
    //     .onTrue(nonAmpTravelNotes);
    //   // Amp Command
    //   new JoystickButton(xboxController, XboxController.Button.kX.value)
    //       .onTrue(new AmpCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // // Climb Prep
    // new JoystickButton(xboxController, XboxController.Button.kStart.value).onTrue(new
    // PrepClimbCommand(robotStateSubsystem, climbSubsystem, superStructure));

  }

  private void configureDriverBindings() {
    FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);

    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(),
            () -> flysky.getStr(),
            () -> flysky.getYaw(),
            driveSubsystem,
            robotStateSubsystem));

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));

    // Intake
    new JoystickButton(driveJoystick, Button.SWB_DWN.id)
        .onTrue(
            new IntakeCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWB_UP.id)
        .onTrue(
            new IntakeCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    // XLock
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new XLockCommand(driveSubsystem))
        .onFalse(new XLockCommand(driveSubsystem));

    // Stow Command
    new JoystickButton(driveJoystick, Button.SWA.id)
        .onTrue(
            new PostClimbStowCommand(
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem,
                climbSubsystem))
        .onFalse(
            new PostClimbStowCommand(
                robotStateSubsystem,
                superStructure,
                magazineSubsystem,
                intakeSubsystem,
                climbSubsystem));

    // // Vision Shoot
    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Release Game Piece Command
    new JoystickButton(driveJoystick, Button.M_SWE.id)
        .onTrue(new ReleaseNoteCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Subwoofer Shoot
    new JoystickButton(driveJoystick, Button.SWG_UP.id)
        .onTrue(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_UP.id)
        .onFalse(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_DWN.id)
        .onTrue(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_DWN.id)
        .onFalse(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));
  }

  public void configureClimbTestBindings() {
    // Open Loop Wrist
    new Trigger((() -> xboxController.getLeftY() > RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopWristCommand(wristSubsystem, -0.2))
        .onFalse(new OpenLoopWristCommand(wristSubsystem, 0.0));
    new Trigger((() -> xboxController.getLeftY() < -RobotConstants.kJoystickDeadband))
        .onTrue(new OpenLoopWristCommand(wristSubsystem, 0.2))
        .onFalse(new OpenLoopWristCommand(wristSubsystem, 0.0));

    // Closed Loop Elbow
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kJoystickDeadband))
        .onTrue(new JogElbowClosedLoopCommand(0.25, elbowSubsystem))
        .onFalse(new HoldElbowCommand(elbowSubsystem));
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kJoystickDeadband))
        .onTrue(new JogElbowClosedLoopCommand(-0.25, elbowSubsystem))
        .onFalse(new HoldElbowCommand(elbowSubsystem));

    // Climb Jog
    new Trigger((() -> xboxController.getLeftTriggerAxis() > RobotConstants.kJoystickDeadband))
        .onTrue(new JogClimbClosedLoopCommand(0.5, climbSubsystem))
        .onFalse(new HoldClimbCommand(climbSubsystem));
    new Trigger((() -> xboxController.getRightTriggerAxis() > RobotConstants.kJoystickDeadband))
        .onTrue(new JogClimbClosedLoopCommand(-0.5, climbSubsystem))
        .onFalse(new HoldClimbCommand(climbSubsystem));

    // Trap Bar

    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new ToggleTrapBarPosCommand(climbSubsystem));

    // Ratchet
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new ToggleRatchetCommand(climbSubsystem));

    // Eject Trap
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new OpenLoopMagazineCommand(magazineSubsystem, -0.2))
        .onFalse(new OpenLoopMagazineCommand(magazineSubsystem, 0.0));

    // // Zero
    // new JoystickButton(xboxController, XboxController.Button.kY.value)
    //     .onTrue(new ZeroClimbCommand(climbSubsystem));

    // Open Loop Forks
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new ForkOpenLoopCommand(climbSubsystem, 0.3))
        .onFalse(new ForkOpenLoopCommand(climbSubsystem, 0.0));
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ForkOpenLoopCommand(climbSubsystem, -0.3))
        .onFalse(new ForkOpenLoopCommand(climbSubsystem, 0.0));

    // Elbow at zero
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(new ClosedLoopElbowCommand(elbowSubsystem, 0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
