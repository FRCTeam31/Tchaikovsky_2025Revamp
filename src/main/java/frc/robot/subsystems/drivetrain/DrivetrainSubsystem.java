package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriverDashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.drivetrain.IDrivetrainIO.*;
import java.util.Map;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.PulsePattern;
import prime.control.LEDs.Patterns.SolidPattern;
import prime.control.SwerveControlSuppliers;

public class DrivetrainSubsystem extends SubsystemBase {

  private PwmLEDs m_leds;

  // Shuffleboard Drivetrain tab configuration
  private DriverDashboard m_driverDashboard;
  private ShuffleboardTab d_drivetrainTab = Shuffleboard.getTab("Drivetrain");
  public GenericEntry d_snapToEnabledEntry = d_drivetrainTab
    .add("SnapTo Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0, 0)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry d_snapAngle = d_drivetrainTab
    .add("SnapTo Angle", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(2, 0)
    .withSize(4, 5)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();

  // IO and swerve modules
  private IDrivetrainIO m_driveio;
  private DrivetrainIOInputs m_inputs;
  private DrivetrainIOOutputs m_outputs;

  // Vision, Kinematics, odometry
  public Limelight LimelightRear;
  public Limelight LimelightFront;
  public boolean EnableContinuousPoseEstimationFront = true;
  public boolean EnableContinuousPoseEstimationRear = true;

  /**
   * Creates a new Drivetrain.
   */
  public DrivetrainSubsystem(boolean isReal, PwmLEDs leds, DriverDashboard driverDashboard) {
    setName("Drivetrain");
    m_leds = leds;
    m_driverDashboard = driverDashboard;

    // Create IO
    m_driveio = isReal ? new DrivetrainIOReal() : new DrivetrainIOSim();
    m_inputs = m_driveio.getInputs();
    m_outputs = new DrivetrainIOOutputs();

    LimelightRear = new Limelight(DriveMap.LimelightRearName);
    LimelightFront = new Limelight(DriveMap.LimelightFrontName);

    // ==================================== PATHPLANNER 2025 ====================================
    // Load the RobotConfig from the GUI settings, or use the default if an exception occurs
    RobotConfig config = DriveMap.PathPlannerRobotConfiguration;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Set up PP to feed current path poses to the field widget
    PathPlannerLogging.setLogCurrentPoseCallback(pose -> driverDashboard.FieldWidget.setRobotPose(pose));
    PathPlannerLogging.setLogTargetPoseCallback(pose ->
      driverDashboard.FieldWidget.getObject("target pose").setPose(pose)
    );
    PathPlannerLogging.setLogActivePathCallback(poses -> driverDashboard.FieldWidget.getObject("path").setPoses(poses));

    // Configure PathPlanner holonomic control
    AutoBuilder.configure(
      this::getPose,
      this::setEstimatorPose,
      this::getRobotRelativeChassisSpeeds,
      (speeds, feedForwards) -> drivePathPlanner(speeds),
      new PPHolonomicDriveController(
        DriveMap.PathingTranslationPid.toPIDConstants(),
        DriveMap.PathingRotationPid.toPIDConstants()
      ),
      config,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        var alliance = DriverStation.getAlliance();

        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
      },
      this
    );
    // Example of overriding PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(() -> m_inputs.SnapCorrectionRadiansPerSecond);
  }

  //#region Control methods

  // Resets the Gyro
  public void resetGyro() {
    m_driveio.resetGyro();
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds) {
    m_outputs.ControlMode = DriveControlMode.kRobotRelative;
    m_outputs.DesiredChassisSpeeds = desiredChassisSpeeds;
  }

  private void drivePathPlanner(ChassisSpeeds pathSpeeds) {
    m_outputs.ControlMode = DriveControlMode.kPathFollowing;
    m_outputs.DesiredChassisSpeeds = pathSpeeds;
  }

  /**
   * Gets the current pose of the drivetrain from odometry
   */
  private Pose2d getPose() {
    return m_inputs.EstimatedRobotPose;
  }

  /**
   * Resets the position of odometry to the input pose
   * @param pose The pose to reset the estimator to
   */
  private void setEstimatorPose(Pose2d pose) {
    m_driveio.setEstimatorPose(pose);
  }

  /**
   * Gets the direction the robot is facing in degrees, CCW+
   */
  private double getHeadingDegrees() {
    return m_inputs.GyroAngle.getDegrees();
  }

  /**
   * Enabled/disables snap-to control
   */
  private void setSnapToEnabled(boolean enabled) {
    m_outputs.SnapEnabled = enabled;
    if (!enabled) m_leds.restorePersistentStripPattern();
  }

  /**
   * Sets the snap-to gyro setpoint, converting from degrees to radians
   * @param angle The angle to snap to in degrees
   */
  private void setSnapToSetpoint(double angle) {
    var setpoint = MathUtil.angleModulus(Rotation2d.fromDegrees(angle).getRadians());

    m_outputs.SnapEnabled = true;
    m_outputs.SnapSetpoint = Rotation2d.fromRadians(setpoint);
  }

  /**
   * Gets the current chassis speeds of the robot
   */
  private ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return m_inputs.RobotRelativeChassisSpeeds;
  }

  /**
   * Evaluates the pose estimation using the limelight cameras
   */
  private void evaluatePoseEstimation() {
    var currentSpeeds = getRobotRelativeChassisSpeeds();

    var withinTrustedVelocity =
      currentSpeeds.omegaRadiansPerSecond < 0.2 && // 1 rad/s is about 60 degrees/s
      currentSpeeds.vxMetersPerSecond < 2 &&
      currentSpeeds.vyMetersPerSecond < 2;
    SmartDashboard.putBoolean("Drive/PoseEstimation/WithinTrustedVelocity", withinTrustedVelocity);

    EnableContinuousPoseEstimationRear = m_driverDashboard.RearPoseEstimationSwitch.getBoolean(false);
    SmartDashboard.putBoolean("Drive/PoseEstimation/RearEstimationEnabled", EnableContinuousPoseEstimationRear);
    if (EnableContinuousPoseEstimationRear) {
      // Rear Limelight
      // If we have a valid target and we're moving in a trusted velocity range, update the pose estimator
      var primaryTarget = LimelightRear.getApriltagId();
      var isValidTarget = LimelightRear.isValidApriltag(primaryTarget);
      SmartDashboard.putBoolean("Drive/PoseEstimation/Rear/IsValidTarget", isValidTarget);

      m_driverDashboard.RearApTagIdField.setDouble(primaryTarget);
      m_driverDashboard.RearApTagOffsetDial.setDouble(LimelightRear.getHorizontalOffsetFromTarget().getDegrees());

      if (isValidTarget && withinTrustedVelocity) {
        var llPose = LimelightRear.getRobotPose(Alliance.Blue);

        m_driveio.addPoseEstimatorVisionMeasurement(llPose.Pose.toPose2d(), llPose.Timestamp, llPose.StdDeviations);
      }
    }

    EnableContinuousPoseEstimationFront = m_driverDashboard.FrontPoseEstimationSwitch.getBoolean(false);
    SmartDashboard.putBoolean("Drive/PoseEstimation/FrontEstimationEnabled", EnableContinuousPoseEstimationFront);
    if (EnableContinuousPoseEstimationFront) {
      // Front Limelight
      // If we have a valid target and we're moving in a trusted velocity range, update the pose estimator
      var frontPrimaryTarget = LimelightFront.getApriltagId();
      var frontIsValidTarget = LimelightFront.isValidApriltag(frontPrimaryTarget);
      m_driverDashboard.FrontApTagIdField.setDouble(frontPrimaryTarget);
      SmartDashboard.putBoolean("Drive/PoseEstimation/Front/IsValidTarget", frontIsValidTarget);

      if (frontIsValidTarget && withinTrustedVelocity) {
        var llPose = LimelightFront.getRobotPose(Alliance.Blue);

        m_driveio.addPoseEstimatorVisionMeasurement(llPose.Pose.toPose2d(), llPose.Timestamp, llPose.StdDeviations);
      }
    }
  }

  //#endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Pose estimation
    evaluatePoseEstimation();

    // IO
    m_inputs = m_driveio.getInputs();
    m_driveio.setOutputs(m_outputs);

    // Update LEDs
    if (m_inputs.SnapOnTarget) {
      m_leds.setStripTemporaryPattern(new SolidPattern(Color.GREEN));
    } else {
      m_leds.setStripTemporaryPattern(new PulsePattern(Color.RED, 0.5));
    }

    // Update shuffleboard
    m_driverDashboard.HeadingGyro.setDouble(m_inputs.GyroAngle.getDegrees());
    m_driverDashboard.FieldWidget.setRobotPose(m_inputs.EstimatedRobotPose);
    d_snapToEnabledEntry.setBoolean(m_outputs.SnapEnabled);
    d_snapAngle.setDouble(m_outputs.SnapSetpoint.getRadians());
  }

  //#region Commands

  /**
   * Creates a command that drives the robot using input controls
   * @param controlSuppliers Controller input suppliers
   */
  public Command defaultDriveCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
        // If the driver is trying to rotate the robot, disable snap-to control
        if (Math.abs(controlSuppliers.Z.getAsDouble()) > 0.2) {
          setSnapToEnabled(false);
          m_leds.restorePersistentStripPattern();
        }

        // Convert inputs to MPS
        var inputXMPS = controlSuppliers.X.getAsDouble() * DriveMap.MaxSpeedMetersPerSecond;
        var inputYMPS = -controlSuppliers.Y.getAsDouble() * DriveMap.MaxSpeedMetersPerSecond;
        var inputRotationRadiansPS = -controlSuppliers.Z.getAsDouble() * DriveMap.MaxAngularSpeedRadians;

        // Build chassis speeds
        ChassisSpeeds robotRelativeSpeeds;
        var invert = Robot.onRedAlliance() ? -1 : 1;

        // Drive the robot with the driver-relative inputs, converted to field-relative based on which side we're on
        robotRelativeSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
            (inputYMPS * invert), // Use Y as X for field-relative
            (inputXMPS * invert), // Use X as Y for field-relative
            inputRotationRadiansPS,
            m_inputs.GyroAngle
          );

        driveRobotRelative(robotRelativeSpeeds);
      });
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(() -> resetGyro());
  }

  /**
   * Enables snap-to control and sets an angle setpoint
   * @param angle
   */
  public Command setSnapToSetpointCommand(double angle) {
    return Commands.runOnce(() -> setSnapToSetpoint(Robot.onBlueAlliance() ? angle + 180 : angle));
  }

  /**
   * Disables snap-to control
   */
  public Command disableSnapToCommand() {
    return Commands.runOnce(() -> setSnapToEnabled(false));
  }

  /**
   * Enables lock-on control
   */
  public Command enableLockOn() {
    return Commands.run(() -> {
      var targetedAprilTag = LimelightRear.getApriltagId();

      // If targetedAprilTag is in validTargets, snap to its offset
      if (LimelightRear.isSpeakerCenterTarget(targetedAprilTag)) {
        // Calculate the target heading
        var horizontalOffsetDeg = LimelightRear.getHorizontalOffsetFromTarget().getDegrees();
        var robotHeadingDeg = getHeadingDegrees();
        var targetHeadingDeg = robotHeadingDeg - horizontalOffsetDeg;

        // Set the drivetrain to snap to the target heading
        setSnapToSetpoint(targetHeadingDeg);
      } else {
        setSnapToEnabled(false);
      }
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of("Enable_Lock_On", enableLockOn(), "Disable_Snap_To", disableSnapToCommand());
  }
  //#endregion
}
