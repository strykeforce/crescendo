// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ch.qos.logback.classic.util.ContextInitializer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.RobotConstants;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.slf4j.LoggerFactory;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  org.slf4j.Logger txtLogger;
  private DigitalInput eventFlag;
  private Boolean isEvent;
  private boolean hasAlliance = false;
  private static org.slf4j.Logger logger;

  @Override
  public void robotInit() {
    logger = LoggerFactory.getLogger(Robot.class);
    if (isReal()) {
      Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
      Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
      Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
      Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
      Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
      switch (BuildConstants.DIRTY) {
        case 0:
          Logger.recordMetadata("GitDirty", "All Changes Committed");
          break;
        case 1:
          Logger.recordMetadata("GitDirty", "Uncommitted changes");
          break;
        default:
          Logger.recordMetadata("GitDirty", "Unknown");
          break;
      }
      // /media/sda1/logs
      Logger.addDataReceiver(new WPILOGWriter());

      // Comp robot conditions or not
      eventFlag = new DigitalInput(RobotConstants.kEventInterlockID);
      isEvent = false; // eventFlag.get();
      if (isEvent) {
        System.setProperty(ContextInitializer.CONFIG_FILE_PROPERTY, "logback-event.xml");
        System.out.println("Event Flag Removed - logging to file in ~lvuser/logs/");
      } else {
        Logger.addDataReceiver(new NT4Publisher()); // publish advantage kit data to network tables
      }
    } else {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog(); // pull replay log from advantage scope
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(
          new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // save outputs to new log
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
    // m_robotContainer.setIsEvent(isEvent);
    if (!isEvent) {
      m_robotContainer.configureTelemetry();
      // m_robotContainer.configurePitDashboard();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (!hasAlliance) {
      try {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Blue || alliance == Alliance.Red) {
          hasAlliance = true;
          // m_robotContainer.setAllianceColor(alliance);
          logger.info("Set Alliance to {}", alliance);
        }
      } catch (NoSuchElementException error) {
        // logger.info("Error: {}", error.toString());
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (!m_robotContainer.hasElbowZeroed()) m_robotContainer.zeroElbow();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (!m_robotContainer.hasElbowZeroed()) {
      m_robotContainer.zeroElbow();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
