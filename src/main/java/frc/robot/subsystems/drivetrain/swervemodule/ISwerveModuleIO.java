package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ISwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {

    public SwerveModulePosition ModulePosition = new SwerveModulePosition();
    public SwerveModuleState ModuleState = new SwerveModuleState();
  }

  @AutoLog
  public static class SwerveModuleIOOutputs {

    public SwerveModuleState DesiredState = new SwerveModuleState();
  }

  public SwerveModuleIOInputs getInputs();

  public void setOutputs(SwerveModuleIOOutputs outputs);

  public void stopMotors();
}
