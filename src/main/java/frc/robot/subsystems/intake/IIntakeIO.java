package frc.robot.subsystems.intake;

public interface IIntakeIO {
  public IntakeIOInputs getInputs();

  public void setOutputs(IntakeIOOutputs outputs);

  public void stopAngleMotors();
  
  public void stopRollerMotors();
}
