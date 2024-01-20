// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  VisionSubsystem visionSubsystem;
  DriveSubsystem driveSubsystem;

  public RobotContainer() {
    DriveSubsystem driveSubsystem = new DriveSubsystem();
    VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
