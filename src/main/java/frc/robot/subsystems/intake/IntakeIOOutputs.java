package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;

public class IntakeIOOutputs {
    
  public boolean m_angleToggledIn; //output
  public double leftMotorOutput = 0;
  public double rightMotorOutput = 0;
  public double rollerMotorOutput = 0;
}
