package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.intake.IntakeIOInputs;
import frc.robot.subsystems.intake.IntakeIOOutputs;

public interface IIntakeIO {
      @Logged(name = "Inputs", importance = Logged.Importance.CRITICAL)
  public IntakeIOInputs getInputs();

  public void setOutputs(IntakeIOOutputs outputs);

  public void stopMotors();
}
