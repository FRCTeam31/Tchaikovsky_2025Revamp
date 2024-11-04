package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.swervemodule.*;

@Logged(strategy = Strategy.OPT_IN)
public class DrivetrainIOReal implements IDrivetrainIO {

  private DrivetrainIOInputs m_inputs;

  private Pigeon2 m_gyro;

  @Logged(name = "SnapAnglePIDController", importance = Logged.Importance.CRITICAL)
  private PIDController m_snapAngleController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private ISwerveModuleIO m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;

  @Logged(name = "FrontLeftInputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOInputs m_frontLeftInputs;
  @Logged(name = "FrontRightInputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOInputs m_frontRightInputs;
  @Logged(name = "RearLeftInputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOInputs m_rearLeftInputs;
  @Logged(name = "RearRightInputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOInputs m_rearRightInputs;
  
  @Logged(name = "FrontLeftOutputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOOutputs m_frontLeftOutputs;
  @Logged(name = "FrontRightOutputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOOutputs m_frontRightOutputs;
  @Logged(name = "RearLeftOutputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOOutputs m_rearLeftOutputs;
  @Logged(name = "RearRightOutputs", importance = Logged.Importance.CRITICAL)
  private SwerveModuleIOOutputs m_rearRightOutputs;

  public DrivetrainIOReal() {
    m_inputs = new DrivetrainIOInputs();

    // Create gyro
    m_gyro = new Pigeon2(DriveMap.PigeonId);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());

    // Create swerve modules in CCW order from FL to FR
    m_frontLeftModule = new SwerveModuleIOReal(DriveMap.FrontLeftSwerveModule);
    m_frontRightModule = new SwerveModuleIOReal(DriveMap.FrontRightSwerveModule);
    m_rearLeftModule = new SwerveModuleIOReal(DriveMap.RearLeftSwerveModule);
    m_rearRightModule = new SwerveModuleIOReal(DriveMap.RearRightSwerveModule);

    // Configure snap-to PID
    m_snapAngleController = DriveMap.SnapToPID.createPIDController(0.02);
    m_snapAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Create kinematics in order FL, FR, RL, RR
    m_kinematics =
      new SwerveDriveKinematics(
        DriveMap.FrontLeftSwerveModule.ModuleLocation,
        DriveMap.FrontRightSwerveModule.ModuleLocation,
        DriveMap.RearLeftSwerveModule.ModuleLocation,
        DriveMap.RearRightSwerveModule.ModuleLocation
      );

    // Create pose estimator
    m_poseEstimator =
      new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), getModulePositions(), new Pose2d());
  }

  @Override
  public DrivetrainIOInputs getInputs() {
    m_frontLeftInputs = m_frontLeftModule.getInputs();
    m_frontRightInputs = m_frontRightModule.getInputs();
    m_rearLeftInputs = m_rearLeftModule.getInputs();
    m_rearRightInputs = m_rearRightModule.getInputs();

    m_inputs.GyroAngle = m_gyro.getRotation2d();
    m_inputs.GyroAccelX = m_gyro.getAccelerationX().getValueAsDouble();
    m_inputs.GyroAccelY = m_gyro.getAccelerationY().getValueAsDouble();
    m_inputs.GyroAccelZ = m_gyro.getAccelerationZ().getValueAsDouble();
    m_inputs.RobotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    var modulePositions = getModulePositions();
    m_inputs.EstimatedRobotPose = m_poseEstimator.update(m_inputs.GyroAngle, modulePositions);

    return m_inputs;
  }

  @Override
  public void setOutputs(DrivetrainIOOutputs outputs) {
    if (outputs.SnapEnabled) {
      m_snapAngleController.setSetpoint(outputs.SnapSetpoint.getRadians());
    }

    switch (outputs.ControlMode) {
      case kRobotRelative:
        driveRobotRelative(outputs.DesiredChassisSpeeds, outputs.SnapEnabled);
        break;
      case kFieldRelative:
        drivePathPlanner(outputs.DesiredChassisSpeeds);
        break;
      default:
        break;
    }

    m_frontLeftModule.setOutputs(m_frontLeftOutputs);
    m_frontRightModule.setOutputs(m_frontRightOutputs);
    m_rearLeftModule.setOutputs(m_rearLeftOutputs);
    m_rearRightModule.setOutputs(m_rearRightOutputs);
  }

  /**
   * Resets the gyro to 0 or 180 degrees based on the alliance
   */
  @Override
  public void resetGyro() {
    m_gyro.setYaw(Robot.onBlueAlliance() ? 180 : 0);

    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), m_poseEstimator.getEstimatedPosition());
  }

  /**
   * Explicitly sets the pose estimator's pose (used when autonomous begins)
   */
  @Override
  public void setEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator
   */
  @Override
  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviations);
  }

  /**
   * Stops all module motors
   */
  @Override
  public void stopAllMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean snapAngleEnabled) {
    // If snap-to is enabled, calculate and override the input rotational speed to reach the setpoint
    if (snapAngleEnabled) {
      var currentRotationRadians = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());
      var snapCorrection = m_snapAngleController.calculate(currentRotationRadians);
      m_inputs.SnapCorrectionRadiansPerSecond = snapCorrection;
      desiredChassisSpeeds.omegaRadiansPerSecond = snapCorrection;

      // Report back if snap is on-target
      m_inputs.SnapIsOnTarget = Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < 0.1;
    }

    // Correct drift by taking the input speeds and converting them to a desired per-period speed. This is known as "discretizing"
    desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    setDesiredModuleStates(swerveModuleStates);
  }

  /**
   * Facilitates driving using PathPlanner generated speeds
   * @param robotRelativeSpeeds
   */
  private void drivePathPlanner(ChassisSpeeds robotRelativeSpeeds) {
    if (Robot.onRedAlliance()) {
      // If we're on the red alliance, we need to flip the gyro
      var gyroAngle = m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(180));

      // Convert the robot-relative speeds to field-relative speeds with the flipped gyro
      var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyroAngle);

      // Convert back to robot-relative speeds, also with the flipped gyro
      driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, gyroAngle), false);
    } else {
      driveRobotRelative(robotRelativeSpeeds, false);
    }
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * @param desiredStates
   */
  private void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeftOutputs.DesiredState = desiredStates[0];
    m_frontRightOutputs.DesiredState = desiredStates[1];
    m_rearLeftOutputs.DesiredState = desiredStates[2];
    m_rearRightOutputs.DesiredState = desiredStates[3];
  }

  /**
   * Gets the instantaneous states for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeftInputs.ModuleState,
      m_frontRightInputs.ModuleState,
      m_rearLeftInputs.ModuleState,
      m_rearRightInputs.ModuleState,
    };
  }

  /**
   * Gets the cumulative positions for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeftInputs.ModulePosition,
      m_frontRightInputs.ModulePosition,
      m_rearLeftInputs.ModulePosition,
      m_rearRightInputs.ModulePosition,
    };
  }
}
