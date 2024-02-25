// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climb.ForkOpenLoopCommand;
import frc.robot.commands.climb.HoldClimbCommand;
import frc.robot.commands.climb.JogClimbClosedLoopCommand;
import frc.robot.commands.climb.ToggleRatchetCommand;
import frc.robot.commands.climb.ToggleTrapBarPosCommand;
import frc.robot.commands.climb.ZeroClimbCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.elbow.HoldElbowCommand;
import frc.robot.commands.elbow.JogElbowClosedLoopCommand;
import frc.robot.commands.magazine.OpenLoopMagazineCommand;
import frc.robot.commands.wrist.OpenLoopWristCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
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

  private DriveAutonCommand testAutonPath;
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
            visionSubsystem, driveSubsystem, intakeSubsystem, magazineSubsystem, superStructure);

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    // visionSubsystem.setVisionUpdates(false);
    // testAutonPath = new DriveAutonCommand(driveSubsystem, "5mTestPath", true, true);
    // testAutonPath.generateTrajectory();

    configureDriverBindings();
    // configureOperatorBindings();
    configureClimbTestBindings();
    // configureMatchDashboard();
    // configurePitDashboard();
    // configureTuningDashboard();
    // robotStateSubsystem.setAllianceColor(Alliance.Blue);

    // configureTelemetry();
    // configurePitDashboard();
  }

  /*
  public void configurePitDashboard() {
    Shuffleboard.getTab("Pit")
        .add(
            "Stow",
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem))
        .withSize(1, 1)
        .withPosition(0, 0);
    Shuffleboard.getTab("Pit")
        .add("Elbow Zero Position", new ClosedLoopElbowCommand(elbowSubsystem, 0.0))
        .withSize(1, 1)
        .withPosition(1, 0);
    Shuffleboard.getTab("Pit")
        .add("Zero Elbow", new ZeroElbowCommand(elbowSubsystem))
        .withSize(1, 1)
        .withPosition(2, 0);
    Shuffleboard.getTab("Pit")
        .add("ShootingZeroTest", new DynamicZeroPosCommand(elbowSubsystem, 25.0))
        .withSize(1, 1)
        .withPosition(3, 0);
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Blue)
            .withProperties(Map.of("colorWhenFalse", "blue"))
            .withSize(2, 2)
            .withPosition(0, 0);

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
  */

  public void configureTelemetry() {
    // driveSubsystem.registerWith(telemetryService);
    // visionSubsystem.registerWith(telemetryService);
    // wristSubsystem.registerWith(telemetryService);
    // elbowSubsystem.registerWith(telemetryService);
    // shooterSubsystem.registerWith(telemetryService);
    // superStructure.registerWith(telemetryService);
    climbSubsystem.registerWith(telemetryService);
    // intakeSubsystem.registerWith(telemetryService);
    // magazineSubsystem.registerWith(telemetryService);
    // robotStateSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }
  /*

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

    // new JoystickButton(xboxController, XboxController.Button.kX.value).onTrue(testAutonPath);

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

  */

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

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));

    /*
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
    */
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

    // Zero
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new ZeroClimbCommand(climbSubsystem));

    // Open Loop Forks
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new ForkOpenLoopCommand(climbSubsystem, 0.1))
        .onFalse(new ForkOpenLoopCommand(climbSubsystem, 0.0));
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ForkOpenLoopCommand(climbSubsystem, -0.1))
        .onFalse(new ForkOpenLoopCommand(climbSubsystem, 0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
