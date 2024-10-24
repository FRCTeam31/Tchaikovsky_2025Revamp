// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.DriveMap;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.Map;
import prime.control.Controls;
import prime.control.HolonomicControlStyle;
import prime.control.PrimeXboxController;

public class RobotContainer {

  private PrimeXboxController m_driverController;
  private PrimeXboxController m_operatorController;

  public DrivetrainSubsystem Drivetrain;
  public Shooter Shooter;
  public Intake Intake;
  public Climbers Climbers;
  public PwmLEDs LEDs;
  public Compressor Compressor;
  public DriverDashboard DriverDashboard;

  private CombinedCommands m_combinedCommands;

  public RobotContainer(boolean isReal) {
    try {
      m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
      m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

      // Create new subsystems
      LEDs = new PwmLEDs();
      DriverDashboard = new DriverDashboard();
      Drivetrain = new DrivetrainSubsystem(isReal, LEDs, DriverDashboard);
      Shooter = new Shooter(LEDs);
      Intake = new Intake();
      Climbers = new Climbers(DriverDashboard);
      Compressor = new Compressor(30, PneumaticsModuleType.REVPH);
      Compressor.enableDigital();

      m_combinedCommands = new CombinedCommands();

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());
      NamedCommands.registerCommands(Intake.getNamedCommands());
      NamedCommands.registerCommands(Shooter.getNamedCommands());
      NamedCommands.registerCommands(m_combinedCommands.getNamedCommands()); // Register the combined named commands that use multiple subsystems

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
      Drivetrain.defaultDriveCommand(
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
    m_driverController.y().onTrue(Climbers.toggleClimbControlsCommand());
    m_driverController.start().onTrue(Climbers.setArmsUpCommand());
    Climbers.setDefaultCommand(
      Climbers.defaultClimbingCommand(
        m_driverController.button(Controls.RB),
        m_driverController.button(Controls.LB),
        () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
        () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
      )
    );
  }

  /**
   * Creates the controller and configures the operator's controls
   */
  public void configureOperatorControls() {
    // Intake ========================================
    m_operatorController.a().onTrue(Intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out

    m_operatorController // When the trigger is pressed, intake a note at a variable speed
      .leftTrigger(0.1)
      .whileTrue(Intake.runRollersAtSpeedCommand(() -> m_operatorController.getLeftTriggerAxis()))
      .onFalse(Intake.stopRollersCommand());

    m_operatorController // When the trigger is pressed, eject a note at a constant speed
      .rightTrigger(0.1)
      .whileTrue(Intake.ejectNoteCommand())
      .onFalse(Intake.stopRollersCommand());

    // Shooter ========================================
    m_operatorController // Toggle the elevation of the shooter
      .rightBumper()
      .onTrue(Shooter.toggleElevationCommand());

    m_operatorController // Runs only the shooter motors at a constant speed to score in the amp
      .x()
      .whileTrue(Shooter.startShootingNoteCommand())
      .onFalse(Shooter.stopMotorsCommand());

    // Combined shooter and intake commands ===========
    m_operatorController // score in speaker
      .b()
      .onTrue(m_combinedCommands.scoreInSpeakerSequentialGroup());

    m_operatorController // Run sequence to load a note into the shooter for scoring in the amp
      .y()
      .onTrue(m_combinedCommands.loadNoteForAmp());
  }

  public class CombinedCommands {

    /**
     * Runs a sequence to score a note in the speaker
     * @return
     */
    public SequentialCommandGroup scoreInSpeakerSequentialGroup() {
      return Shooter
        .startShootingNoteCommand()
        .andThen(new WaitCommand(0.75))
        .andThen(Intake.ejectNoteCommand())
        .andThen(new WaitCommand(0.75))
        .andThen(Shooter.stopMotorsCommand())
        .andThen(Intake.stopRollersCommand());
    }

    /**
     * Runs a sequence to load a note into the shooter for scoring in the amp
     * @return
     */
    public SequentialCommandGroup loadNoteForAmp() {
      return Commands
        .runOnce(() -> Intake.runIntakeRollers(-0.7)) // Eject from the intake
        .alongWith(Commands.runOnce(() -> Shooter.runShooter(0.1))) // Load into the shooter
        .andThen(new WaitUntilCommand(Shooter::isNoteLoaded).withTimeout(1)) // Wait until the note is loaded
        .andThen(new WaitCommand(0.045)) // Give the note time to get into the shooter
        .andThen(stopShooterAndIntakeCommand()); // Stop both the shooter and intake
    }

    /**
     * Runs a sequence to stop both the shooter and intake
     * @return
     */
    public SequentialCommandGroup stopShooterAndIntakeCommand() {
      return Shooter.stopMotorsCommand().andThen(Intake.stopRollersCommand());
    }

    /**
     * Returns a map of named commands that use multiple subsystems
     * @return
     */
    public Map<String, Command> getNamedCommands() {
      return Map.of(
        "Score_In_Speaker",
        scoreInSpeakerSequentialGroup(),
        "Load_Note_For_Amp",
        loadNoteForAmp(),
        "Stop_Shooter_And_Intake",
        stopShooterAndIntakeCommand()
      );
    }
  }
}
