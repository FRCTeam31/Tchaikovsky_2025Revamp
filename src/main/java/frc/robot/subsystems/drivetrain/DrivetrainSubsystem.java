package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DriverDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import prime.control.SwerveControlSuppliers;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.LEDPattern;
import prime.control.LEDs.Patterns.PulsePattern;
import prime.control.LEDs.Patterns.SolidPattern;
import prime.vision.LimelightInputs;

@Logged(strategy = Strategy.OPT_IN)
public class DrivetrainSubsystem extends SubsystemBase {

  private Runnable m_restoreLEDPersistentPatternFunc;
  private Consumer<LEDPattern> m_setLEDTemporaryPatternFunc;

  // Shuffleboard Drivetrain tab configuration
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

  // IO
  private IDrivetrainIO m_driveio;

  @Logged(name = "DriveInputs", importance = Logged.Importance.CRITICAL)
  private DrivetrainIOInputs m_inputs;
  @Logged(name = "DriveOutputs", importance = Logged.Importance.CRITICAL)
  private DrivetrainIOOutputs m_outputs;

  // Vision, Kinematics, odometry
  private Supplier<LimelightInputs[]> m_limelightInputsSupplier;
  @Logged(importance = Logged.Importance.CRITICAL)
  public boolean EstimatePoseUsingFrontCamera = true;
  @Logged(importance = Logged.Importance.CRITICAL)
  public boolean EstimatePoseUsingRearCamera = true;
  @Logged(importance = Logged.Importance.CRITICAL)
  public boolean WithinPoseEstimationVelocity = true;

  /**
   * Creates a new Drivetrain.
   */
  public DrivetrainSubsystem(
    boolean isReal, 
    Runnable restoreLEDPersistentPatternFunc,
    Consumer<LEDPattern> setLEDTemporaryPatternFunc,
    Supplier<LimelightInputs[]> limelightInputsSupplier) {
    setName("Drivetrain");

    m_restoreLEDPersistentPatternFunc = restoreLEDPersistentPatternFunc;
    m_setLEDTemporaryPatternFunc = setLEDTemporaryPatternFunc;

    // Create IO
    m_driveio = isReal 
      ? new DrivetrainIOReal() 
      : new DrivetrainIOSim();
    m_inputs = m_driveio.getInputs();
    m_outputs = new DrivetrainIOOutputs();

    m_limelightInputsSupplier = limelightInputsSupplier;

    configurePathPlanner();
  }

  private void configurePathPlanner() {
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
    PathPlannerLogging.setLogCurrentPoseCallback(pose -> DriverDashboard.FieldWidget.setRobotPose(pose));
    PathPlannerLogging.setLogTargetPoseCallback(pose ->
      DriverDashboard.FieldWidget.getObject("target pose").setPose(pose)
    );
    PathPlannerLogging.setLogActivePathCallback(poses -> DriverDashboard.FieldWidget.getObject("path").setPoses(poses));

    // Configure PathPlanner holonomic control
    AutoBuilder.configure(
      () -> m_inputs.EstimatedRobotPose,
      m_driveio::setEstimatorPose,
      () -> m_inputs.RobotRelativeChassisSpeeds,
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
    m_outputs.ControlMode = DrivetrainControlMode.kRobotRelative;
    m_outputs.DesiredChassisSpeeds = desiredChassisSpeeds;
  }

  private void drivePathPlanner(ChassisSpeeds pathSpeeds) {
    m_outputs.ControlMode = DrivetrainControlMode.kPathFollowing;
    m_outputs.DesiredChassisSpeeds = pathSpeeds;
  }

  /**
   * Enabled/disables snap-to control
   */
  private void setSnapToEnabled(boolean enabled) {
    m_outputs.SnapEnabled = enabled;
    if (!enabled) m_restoreLEDPersistentPatternFunc.run();
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
   * Evaluates the pose estimation using the limelight cameras
   */
  private void evaluatePoseEstimation(boolean withinTrustedVelocity, int limelightIndex) {

    // If we have a valid target and we're moving in a trusted velocity range, update the pose estimator
    var limelightInputs = m_limelightInputsSupplier.get()[limelightIndex];
    var isValidTarget = VisionSubsystem.isAprilTagIdValid(limelightInputs.ApriltagId);
    if (isValidTarget && withinTrustedVelocity) {
      var llPose = limelightInputs.BlueAllianceOriginFieldSpaceRobotPose;

      m_driveio.addPoseEstimatorVisionMeasurement(llPose.Pose.toPose2d(), llPose.Timestamp, llPose.StdDeviations);
    }
  }

  //#endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Get inputs
    m_inputs = m_driveio.getInputs();

    // Pose estimation
    WithinPoseEstimationVelocity =
      m_inputs.RobotRelativeChassisSpeeds.omegaRadiansPerSecond < 0.2 && // 1 rad/s is about 60 degrees/s
      m_inputs.RobotRelativeChassisSpeeds.vxMetersPerSecond < 2 &&
      m_inputs.RobotRelativeChassisSpeeds.vyMetersPerSecond < 2;

    EstimatePoseUsingFrontCamera = DriverDashboard.FrontPoseEstimationSwitch.getBoolean(false);
    if (EstimatePoseUsingFrontCamera) evaluatePoseEstimation(WithinPoseEstimationVelocity, 0);

    EstimatePoseUsingRearCamera = DriverDashboard.RearPoseEstimationSwitch.getBoolean(false);
    if (EstimatePoseUsingRearCamera) evaluatePoseEstimation(WithinPoseEstimationVelocity, 1);

    // Update LEDs
    if (m_inputs.SnapOnTarget) {
      m_setLEDTemporaryPatternFunc.accept(new SolidPattern(Color.GREEN));
    } else {
      m_setLEDTemporaryPatternFunc.accept(new PulsePattern(Color.RED, 0.5));
    }

    // Send outputs to the drive IO
    m_driveio.setOutputs(m_outputs);

    // Update shuffleboard
    DriverDashboard.HeadingGyro.setDouble(m_inputs.GyroAngle.getDegrees());
    DriverDashboard.FieldWidget.setRobotPose(m_inputs.EstimatedRobotPose);
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
          m_restoreLEDPersistentPatternFunc.run();
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
      var rearLimelightInputs = m_limelightInputsSupplier.get()[1];

      // If targeted AprilTag is in validTargets, snap to its offset
      if (VisionSubsystem.isAprilTagIdASpeakerCenterTarget(rearLimelightInputs.ApriltagId)) {
        // Calculate the target heading
        var horizontalOffsetDeg = rearLimelightInputs.TargetHorizontalOffset.getDegrees();
        var robotHeadingDeg = m_inputs.GyroAngle.getDegrees();
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
