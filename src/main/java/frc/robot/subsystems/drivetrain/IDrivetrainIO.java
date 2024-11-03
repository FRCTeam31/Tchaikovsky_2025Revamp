package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@Logged(strategy = Strategy.OPT_IN)
public interface IDrivetrainIO {

  @Logged(name = "Inputs", importance = Logged.Importance.CRITICAL)
  public DrivetrainIOInputs getInputs();

  public void setOutputs(DrivetrainIOOutputs outputs);

  public void resetGyro();

  public void setEstimatorPose(Pose2d pose);

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations);

  public void stopAllMotors();

  @Logged(name = "ModuleStates", importance = Logged.Importance.CRITICAL)
  public SwerveModuleState[] getModuleStates();

  @Logged(name = "ModulePositions", importance = Logged.Importance.CRITICAL)
  public SwerveModulePosition[] getModulePositions();
}
