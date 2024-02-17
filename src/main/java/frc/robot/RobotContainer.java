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
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.HoloContTuningCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.SetGyroOffsetCommand;
import frc.robot.commands.drive.ToggleVisionUpdatesCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.elbow.OpenLoopElbowCommand;
import frc.robot.commands.robotState.AmpCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.commands.robotState.PodiumCommand;
import frc.robot.commands.robotState.ReleaseNoteCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.SubWooferCommand;
import frc.robot.commands.robotState.TuningOffCommand;
import frc.robot.commands.robotState.TuningShootCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.commands.wrist.OpenLoopWristCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.climb.ClimbSubsystem;
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

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);

  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private Alliance alliance = Alliance.Blue;
  private SuppliedValueWidget<Boolean> allianceColor;
  private Boolean isEvent = true;

  //   private DriveAutonCommand testAutonPath;
  private HoloContTuningCommand holoContTuningCommand;
  public GenericEntry lShooterSpeed;
  public GenericEntry rShooterSpeed;
  public GenericEntry magazineSpeed;
  public GenericEntry elbowPos;
  public GenericEntry duplicateShooters;

  public RobotContainer() {
    robotConstants = new RobotConstants();
    driveSubsystem = new DriveSubsystem(new Swerve());
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    wristSubsystem = new WristSubsystem(new WristIOSRX());
    elbowSubsystem = new ElbowSubsystem(new ElbowIOFX());
    shooterSubsystem = new ShooterSubsystem(new ShooterIOFX());
    climbSubsystem = new ClimbSubsystem();
    intakeSubsystem = new IntakeSubsystem(new IntakeIOFX());
    magazineSubsystem = new MagazineSubsystem(new MagazineIOFX());

    intakeSubsystem.setFwdLimitSwitchSupplier(driveSubsystem.getAzimuth1FwdLimitSupplier());

    superStructure =
        new SuperStructure(wristSubsystem, elbowSubsystem, shooterSubsystem, magazineSubsystem);

    robotStateSubsystem =
        new RobotStateSubsystem(
            visionSubsystem, driveSubsystem, intakeSubsystem, magazineSubsystem, superStructure);

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    // visionSubsystem.setVisionUpdates(false);
    // testAutonPath = new DriveAutonCommand(driveSubsystem, "5mTestPath", true, true);
    // testAutonPath.generateTrajectory();

    holoContTuningCommand = new HoloContTuningCommand(driveSubsystem);
    holoContTuningCommand.generateTrajectory();

    configureDriverBindings();
    configureOperatorBindings();
    configureMatchDashboard();
    configurePitDashboard();
    configureTuningDashboard();
    // robotStateSubsystem.setAllianceColor(Alliance.Blue);

    // configureTelemetry();
    // configurePitDashboard();
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
        .add("Lock Wheels Zero", new LockZeroCommand(driveSubsystem))
        .withSize(1, 1)
        .withPosition(1, 0);
    Shuffleboard.getTab("Pit")
        .add(
            "Set Gyro offset -60",
            new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(-60)))
        .withSize(1, 1)
        .withPosition(3, 0);
  }

  private void configureMatchDashboard() {
    Shuffleboard.getTab("Match")
        .add(new ToggleVisionUpdatesCommand(driveSubsystem))
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Vision updates enabled", () -> driveSubsystem.usingVisionUpdates())
        .withSize(1, 1)
        .withPosition(3, 0);

    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Blue)
            .withProperties(Map.of("colorWhenFalse", "blue"))
            .withSize(2, 2)
            .withPosition(1, 0);

    Shuffleboard.getTab("Match")
        .addBoolean("Have Note", () -> robotStateSubsystem.hasNote())
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
            () -> duplicateShooters.getBoolean(true)));
    tab.add(
        "stop",
        new TuningOffCommand(
            robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
  }

  public void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    wristSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);
    shooterSubsystem.registerWith(telemetryService);
    superStructure.registerWith(telemetryService);
    // climbSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    magazineSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);

    // Flips gyro angle if alliance is red team
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
    }
  }

  public void setIsEvent(boolean isEvent) {
    this.isEvent = isEvent;
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

    // Open Loop Magazine
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(
            new AmpCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    // new JoystickButton(xboxController, XboxController.Button.kB.value)
    //     .onTrue(new OpenLoopMagazineCommand(magazineSubsystem, .2))
    //     .onFalse(new OpenLoopMagazineCommand(magazineSubsystem, 0));

    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(holoContTuningCommand);

    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(
            new PodiumCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // SubWoofer
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    //   // Amp Command
    //   new JoystickButton(xboxController, XboxController.Button.kX.value)
    //       .onTrue(new AmpCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    //   // Intake Command
    //   new JoystickButton(xboxController, XboxController.Button.kX.value)
    //       .onTrue(
    //           new IntakeCommand(
    //               robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    //   // Open Loop Intake Command
    //   new JoystickButton(xboxController, XboxController.Button.kA.value)
    //       .onTrue(new OpenLoopIntakeCommand(intakeSubsystem, 50));

    //   new JoystickButton(xboxController, XboxController.Button.kB.value)
    //       .onTrue(new OpenLoopIntakeCommand(intakeSubsystem, 0.0));

    //   // Stow Command
    //   new JoystickButton(xboxController, XboxController.Button.kX.value)
    //       .onTrue(
    //           new StowCommand(
    //               robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
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

    //   // Vision Shoot Command
    //   new JoystickButton(driveJoystick, Button.SWD.id)
    //       .onTrue(new VisionShootCommand(robotStateSubsystem, superStructure,
    // magazineSubsystem));

    //   // Stow Command
    //   new JoystickButton(driveJoystick, Button.SWD.id)
    //       .onTrue(
    //           new StowCommand(
    //               robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Intake
    new JoystickButton(driveJoystick, Button.SWB_DWN.id)
        .onTrue(
            new IntakeCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWB_UP.id)
        .onTrue(
            new IntakeCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));

    // XLock
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new XLockCommand(driveSubsystem))
        .onFalse(new XLockCommand(driveSubsystem));

    new JoystickButton(driveJoystick, Button.SWA.id)
        .onTrue(
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem))
        .onFalse(
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Shoot
    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(new SubWooferCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Release Game Piece Command
    new JoystickButton(driveJoystick, Button.M_SWE.id)
        .onTrue(new ReleaseNoteCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    new JoystickButton(driveJoystick, Button.SWG_UP.id)
        .onTrue(
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_UP.id)
        .onFalse(
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_DWN.id)
        .onTrue(
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_DWN.id)
        .onFalse(
            new VisionShootCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
