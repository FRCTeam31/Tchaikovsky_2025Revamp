package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface IDrivetrainIO {
  @AutoLog
  public static class DrivetrainIOInputs {

    public Rotation2d GyroAngle = new Rotation2d();
    public double GyroAccelX = 0;
    public double GyroAccelY = 0;
    public double GyroAccelZ = 0;
    public boolean SnapOnTarget = false;
    public ChassisSpeeds RobotRelativeChassisSpeeds = new ChassisSpeeds();
    public Pose2d EstimatedRobotPose = new Pose2d();
    public double SnapCorrectionRadiansPerSecond = 0;
  }

  @AutoLog
  public static class DrivetrainIOOutputs {

    public boolean SnapEnabled = false;
    public Rotation2d SnapSetpoint = new Rotation2d();
    public ChassisSpeeds DesiredChassisSpeeds = new ChassisSpeeds();
    public DriveControlMode ControlMode = DriveControlMode.kRobotRelative;
  }

  public enum DriveControlMode {
    kRobotRelative,
    kFieldRelative,
    kPathFollowing,
  }

  public DrivetrainIOInputs getInputs();

  public void setOutputs(DrivetrainIOOutputs outputs);

  public void resetGyro();

  public void setEstimatorPose(Pose2d pose);

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations);

  public void stopAllMotors();
}
