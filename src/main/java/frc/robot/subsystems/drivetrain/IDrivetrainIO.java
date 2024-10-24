package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface IDrivetrainIO {

  public DrivetrainIOInputs getInputs();

  public void setOutputs(DrivetrainIOOutputs outputs);

  public void resetGyro();

  public void setEstimatorPose(Pose2d pose);

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations);

  public void stopAllMotors();
}
