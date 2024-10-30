package frc.robot.subsystems.drivetrain.swervemodule;

public class SwerveModuleIOSim implements ISwerveModuleIO {

  private SwerveModuleIOInputs m_inputs = new SwerveModuleIOInputs();

  public SwerveModuleIOSim(SwerveModuleConfig moduleMap) {
  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    return m_inputs;
  }

  @Override
  public void setOutputs(SwerveModuleIOOutputs outputs) {
    // Right now, just assume that the module is perfect and has no losses.
    // This may be replaced with a more realistic simulation later.
    m_inputs.ModuleState = outputs.DesiredState;
  }

  @Override
  public void stopMotors() {
  }
}
