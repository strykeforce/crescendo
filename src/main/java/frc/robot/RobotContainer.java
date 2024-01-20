// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.robotState.AmpCommand;
import frc.robot.commands.robotState.IntakeCommand;
import frc.robot.commands.robotState.ReleaseNoteCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.VisionShootCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
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

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    wristSubsystem = new WristSubsystem(new WristIOSRX());
    elbowSubsystem = new ElbowSubsystem(new ElbowIOFX());
    shooterSubsystem = new ShooterSubsystem(new ShooterIOFX());
    superStructure = new SuperStructure(wristSubsystem, elbowSubsystem, shooterSubsystem);
    climbSubsystem = new ClimbSubsystem();
    intakeSubsystem = new IntakeSubsystem(new IntakeIOFX());
    magazineSubsystem = new MagazineSubsystem(new MagazineIOFX());

    robotStateSubsystem =
        new RobotStateSubsystem(
            visionSubsystem,
            driveSubsystem,
            shooterSubsystem,
            intakeSubsystem,
            elbowSubsystem,
            magazineSubsystem,
            superStructure);

    configureDriverBindings();
    configureOperatorBindings();
    configureMatchDashboard();

    configureTelemetry();
    configurePitDashboard();
  }

  private void configurePitDashboard() {
    Shuffleboard.getTab("Pit")
        .add(
            "Stow",
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem))
        .withSize(1, 1)
        .withPosition(0, 0);


  }

  private void configureMatchDashboard() {
    Shuffleboard.getTab("Match")
        .addBoolean("Have Note", () -> robotStateSubsystem.hasNote())
        .withSize(1, 1)
        .withPosition(0, 0);

        
  }

  private void configureTelemetry() {
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

  private void configureOperatorBindings() {
    // Amp Command
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new AmpCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Intake Command
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(
            new IntakeCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Stow Command
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));
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

    // Vision Shoot Command
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new VisionShootCommand(robotStateSubsystem, superStructure, magazineSubsystem));

    // Stow Command
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(
            new StowCommand(
                robotStateSubsystem, superStructure, magazineSubsystem, intakeSubsystem));

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.SWD.id).onTrue(new ResetGyroCommand(driveSubsystem));

    // Release Game Piece Command
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new ReleaseNoteCommand(robotStateSubsystem, superStructure, magazineSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
