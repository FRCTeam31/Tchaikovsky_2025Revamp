// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import prime.control.Controls;
import prime.control.HolonomicControlStyle;
import prime.control.PrimeXboxController;

@Logged(strategy = Strategy.OPT_IN)
public class Container {

  private PrimeXboxController m_driverController;
  private PrimeXboxController m_operatorController;

  @Logged(name="Vision", importance = Importance.CRITICAL)
  public VisionSubsystem Vision;
  @Logged(name="Drive", importance = Importance.CRITICAL)
  public DrivetrainSubsystem Drivetrain;
  // public Shooter Shooter;
  // public Intake Intake;
  // public Climbers Climbers;
  // public PwmLEDs LEDs;
  @Logged(name="LEDs", importance = Importance.CRITICAL)
  public PwmLEDs LEDs;
  // public Compressor Compressor;

  private CombinedCommands m_combinedCommands;

  public Container(boolean isReal) {
    try {
      DriverDashboard.init(isReal);
      m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
      // m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

      // Create new subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Drivetrain = new DrivetrainSubsystem(isReal, 
        LEDs::clearForegroundPattern, 
        LEDs::setForegroundPattern, 
        Vision::getAllLimelightInputs);
      // Shooter = new Shooter(
      //   LEDs::clearForegroundPattern,
      //   LEDs::setForegroundPattern);
      // Intake = new Intake();
      // Climbers = new Climbers();
      // Compressor = new Compressor(30, PneumaticsModuleType.REVPH);
      // Compressor.enableDigital();

      m_combinedCommands = new CombinedCommands();

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());
      // NamedCommands.registerCommands(Intake.getNamedCommands());
      // NamedCommands.registerCommands(Shooter.getNamedCommands());
      // NamedCommands.registerCommands(m_combinedCommands.getNamedCommands(Shooter, Intake)); // Register the combined named commands that use multiple subsystems

      // Create Auto chooser and Auto tab in Shuffleboard
      configAutonomousDashboardItems();

      // Reconfigure bindings
      configureDriverControls();
      configureOperatorControls();
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to configure robot: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Configures the autonomous dashboard items
   */
  public void configAutonomousDashboardItems() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    DriverDashboard.addAutoChooser(AutoBuilder.buildAutoChooser("Straight Park"));

    // Add all autos to the auto tab
    var possibleAutos = AutoBuilder.getAllAutoNames();
    for (int i = 0; i < possibleAutos.size(); i++) {
      var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
      DriverDashboard.AutoTab.add(possibleAutos.get(i), autoCommand).withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
    }
  }

  /**
   * Returns the selected autonomous command to run
   * @return
   */
  public Command getAutonomousCommand() {
    return DriverDashboard.AutoChooser.getSelected();
  }

  /**
   * Creates the controller and configures the driver's controls
   */
  public void configureDriverControls() {
    // Controls for Driving
    m_driverController.a().onTrue(Drivetrain.resetGyroCommand());
    Drivetrain.setDefaultCommand(
      Drivetrain.driveRobotRelativeCommand(
        m_driverController.getSwerveControlProfile(
          HolonomicControlStyle.Drone,
          DriveMap.DriveDeadband,
          DriveMap.DeadbandCurveWeight
        )
      )
    );

    // While holding b, auto-aim the robot to the apriltag target using snap-to
    m_driverController.leftStick().whileTrue(Drivetrain.enableLockOn()).onFalse(Drivetrain.disableSnapToCommand());

    // Controls for Snap-To with field-relative setpoints
    m_driverController.x().onTrue(Drivetrain.disableSnapToCommand());
    m_driverController.pov(Controls.up).onTrue(Drivetrain.setSnapToSetpointCommand(0));
    m_driverController.pov(Controls.left).onTrue(Drivetrain.setSnapToSetpointCommand(270));
    m_driverController.pov(Controls.down).onTrue(Drivetrain.setSnapToSetpointCommand(180));
    m_driverController.pov(Controls.right).onTrue(Drivetrain.setSnapToSetpointCommand(90));

    // Climbers
    // m_driverController.y().onTrue(Climbers.toggleClimbControlsCommand());
    // m_driverController.start().onTrue(Climbers.setArmsUpCommand());
    // Climbers.setDefaultCommand(
    //   Climbers.defaultClimbingCommand(
    //     m_driverController.button(Controls.RB),
    //     m_driverController.button(Controls.LB),
    //     () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
    //     () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
    //   )
    // );
  }

  /**
   * Creates the controller and configures the operator's controls
   */
  public void configureOperatorControls() {
    // Intake ========================================
    // m_operatorController.a().onTrue(Intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out

    // m_operatorController // When the trigger is pressed, intake a note at a variable speed
    //   .leftTrigger(0.1)
    //   .whileTrue(Intake.runRollersAtSpeedCommand(() -> m_operatorController.getLeftTriggerAxis()))
    //   .onFalse(Intake.stopRollersCommand());

    // m_operatorController // When the trigger is pressed, eject a note at a constant speed
    //   .rightTrigger(0.1)
    //   .whileTrue(Intake.ejectNoteCommand())
    //   .onFalse(Intake.stopRollersCommand());

    // // Shooter ========================================
    // m_operatorController // Toggle the elevation of the shooter
    //   .rightBumper()
    //   .onTrue(Shooter.toggleElevationCommand());

    // m_operatorController // Runs only the shooter motors at a constant speed to score in the amp
    //   .x()
    //   .whileTrue(Shooter.startShootingNoteCommand())
    //   .onFalse(Shooter.stopMotorsCommand());

    // // Combined shooter and intake commands ===========
    // m_operatorController // score in speaker
    //   .b()
    //   .onTrue(m_combinedCommands.scoreInSpeakerSequentialGroup(Shooter, Intake));

    // m_operatorController // Run sequence to load a note into the shooter for scoring in the amp
    //   .y()
    //   .onTrue(m_combinedCommands.loadNoteForAmp(Shooter, Intake));
  }
}