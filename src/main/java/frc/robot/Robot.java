// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;
import prime.control.LEDs.Patterns.PulsePattern;

public class Robot extends LoggedRobot {

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    // Start L2 logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Initialize the robot container
    m_robotContainer = new RobotContainer(isReal());

    // Configure AdvantageKit
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      // we will re-enable this when we have a USB stick
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(0, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  @Override
  public void disabledInit() {
    m_robotContainer.LEDs.setStripPersistentPattern(new PulsePattern(onRedAlliance() ? Color.RED : Color.BLUE, 2));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.DriverDashboard.AllianceBox.setBoolean(onRedAlliance());
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.LEDs.setStripPersistentPattern(new BlinkPattern(onRedAlliance() ? Color.RED : Color.BLUE, 0.250));

    // Cancel any auto command that's still running and reset the subsystem states
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running and set the intake IN
      m_robotContainer.Shooter.stopMotorsCommand().schedule();
      m_robotContainer.Intake.stopRollersCommand().schedule();
      m_robotContainer.Intake.setIntakeInCommand().schedule();
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Exit without scheduling an auto command if none is selected
    if (m_autonomousCommand == null || m_autonomousCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      m_robotContainer.Drivetrain.resetGyro();
    } else {
      // Schedule the auto command
      m_robotContainer.Drivetrain.EnableContinuousPoseEstimationFront = true;
      m_robotContainer.Drivetrain.EnableContinuousPoseEstimationRear = true;

      if (onRedAlliance()) m_robotContainer.Drivetrain.resetGyro();

      SmartDashboard.putString("Robot/Auto/CommandName", m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running
      m_robotContainer.Shooter.stopMotorsCommand().schedule();
      m_robotContainer.Intake.stopRollersCommand().schedule();
    }

    // Set teleop LED pattern
    m_robotContainer.LEDs.setStripPersistentPattern(
      new ChasePattern(onRedAlliance() ? Color.RED : Color.BLUE, 0.5, false)
    );

    m_robotContainer.Drivetrain.EnableContinuousPoseEstimationFront = false;
    m_robotContainer.Drivetrain.EnableContinuousPoseEstimationRear = false;
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public static boolean onRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Blue;
  }
}
