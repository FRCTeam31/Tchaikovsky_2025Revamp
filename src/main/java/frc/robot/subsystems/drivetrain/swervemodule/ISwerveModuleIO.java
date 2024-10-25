package frc.robot.subsystems.drivetrain.swervemodule;

public interface ISwerveModuleIO {

  public SwerveModuleIOInputs getInputs();

  public void setOutputs(SwerveModuleIOOutputs outputs);

  public void stopMotors();
}
