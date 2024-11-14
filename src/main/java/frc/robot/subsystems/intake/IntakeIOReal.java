package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeSubsystem.VMap;

public class IntakeIOReal implements IIntakeIO {
    private DigitalInput m_topLimitSwitch; //input
    private DigitalInput m_bottomLimitSwitch;
    private CANSparkMax m_rollers; //output
    private CANSparkMax m_angleLeft; //output
    private CANSparkMax m_angleRight; //output

    private IntakeIOInputs m_inputs = new IntakeIOInputs();
    

    public IntakeIOReal() {
    m_topLimitSwitch = new DigitalInput(VMap.TopLimitSwitchChannel);
    m_bottomLimitSwitch = new DigitalInput(VMap.BottomLimitSwitchChannel);

    m_rollers = new CANSparkMax(VMap.RollersCanId, MotorType.kBrushless);
    m_rollers.restoreFactoryDefaults();
    m_rollers.setInverted(VMap.RollersInverted);
    m_rollers.setSmartCurrentLimit(40, 50);
    // m_rollers.setOpenLoopRampRate(0.250);

    m_angleLeft = new CANSparkMax(VMap.NeoLeftCanId, MotorType.kBrushless);
    m_angleLeft.restoreFactoryDefaults();
    m_angleLeft.setInverted(VMap.NeoLeftInverted);
    m_angleLeft.setSmartCurrentLimit(40, 60);

    m_angleRight = new CANSparkMax(VMap.NeoRightCanId, MotorType.kBrushless);
    m_angleRight.restoreFactoryDefaults();
    m_angleRight.setInverted(VMap.NeoRightInverted);
    m_angleRight.setSmartCurrentLimit(40, 60);

    m_inputs.AngleStartPoint = getPositionRight();
    SmartDashboard.putNumber("Intake/AngleStartPoint", m_inputs.AngleStartPoint);

    m_inputs.m_anglePid = VMap.IntakeAnglePid.createPIDController(0.02);
    m_anglePid.setSetpoint(m_inputs.m_angleStartPoint);

    // Set the default command for the subsystem so that it runs the PID loop
    // setDefaultCommand(seekAngleSetpointCommand());

    /**
   * Gets the current position of the Intake Angle from the right NEO's encoder
   * @return
   */
  public double getPositionRight() {
    return m_angleRight.getEncoder().getPosition();
  }

  /**
   * Gets the current position of the Intake Angle from the left NEO's encoder
   * @return
   */
  public double getPositionLeft() {
    return m_angleLeft.getEncoder().getPosition();
  }
}