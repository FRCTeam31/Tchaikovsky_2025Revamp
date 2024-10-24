package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class DrivetrainIOSim implements IDrivetrainIO {

  @Override
  public DrivetrainIOInputs getInputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getInputs'");
  }

  @Override
  public void setOutputs(DrivetrainIOOutputs outputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setOutputs'");
  }

  @Override
  public void resetGyro() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetGyro'");
  }

  @Override
  public void setEstimatorPose(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEstimatorPose'");
  }

  @Override
  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'addPoseEstimatorVisionMeasurement'");
  }

  @Override
  public void stopAllMotors() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'addPoseEstimatorVisionMeasurement'");
  }
}
