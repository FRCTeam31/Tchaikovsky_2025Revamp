package frc.robot.subsystems.climbers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.climbers.ClimbersSubsystem.Side;

@Logged(strategy = Strategy.OPT_IN)
public class ClimbersIOReal implements IClimbersIO{
    public DigitalInput m_leftLimitSwitch;
    public DigitalInput m_rightLimitSwitch;
    
    public VictorSPX m_leftVictorSPX;
    public VictorSPX m_rightVictorSPX;
    
    public DoubleSolenoid m_clutchSolenoidLeft;
    public DoubleSolenoid m_clutchSolenoidRight;

    public ClimbersIOReal(){
        m_leftVictorSPX = new VictorSPX(ClimbersSubsystem.VMap.VictorSPXLeftCanID);
    m_leftVictorSPX.configFactoryDefault();
    m_leftVictorSPX.setInverted(ClimbersSubsystem.VMap.LeftInverted);
    m_leftVictorSPX.setNeutralMode(NeutralMode.Brake);
    m_leftVictorSPX.configOpenloopRamp(0.5);

    m_rightVictorSPX = new VictorSPX(ClimbersSubsystem.VMap.VictorSPXRightCanID);
    m_rightVictorSPX.configFactoryDefault();
    m_rightVictorSPX.setInverted(ClimbersSubsystem.VMap.RightInverted);
    m_rightVictorSPX.setNeutralMode(NeutralMode.Brake);
    m_rightVictorSPX.configOpenloopRamp(0.5);

    m_leftLimitSwitch = new DigitalInput(ClimbersSubsystem.VMap.LeftLimitSwitchDIOChannel);
    m_rightLimitSwitch = new DigitalInput(ClimbersSubsystem.VMap.RightLimitSwitchDIOChannel);

    m_clutchSolenoidLeft =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        ClimbersSubsystem.VMap.LeftSolenoidForwardChannel,
        ClimbersSubsystem.VMap.LeftSolenoidReverseChannel
      );
    m_clutchSolenoidRight =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        ClimbersSubsystem.VMap.RightSolenoidForwardChannel,
        ClimbersSubsystem.VMap.RightSolenoidReverseChannel
      );
    }

    public ClimbersIOInputs getInputs() {
        var inputs = new ClimbersIOInputs();
        inputs.leftLimitSwitch = m_leftLimitSwitch.get();
        inputs.rightLimitSwitch = m_rightLimitSwitch.get();
        return inputs;
        
    }

    public void setOutputs(ClimbersIOOutputs outputs){
        m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, outputs.leftMotorOutput);
        m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, outputs.rightMotorOutput);
        m_clutchSolenoidLeft.set(outputs.ClutchLeft ? Value.kForward : Value.kReverse);
        m_clutchSolenoidRight.set(outputs.ClutchRight ? Value.kForward : Value.kReverse);
    }

    
}