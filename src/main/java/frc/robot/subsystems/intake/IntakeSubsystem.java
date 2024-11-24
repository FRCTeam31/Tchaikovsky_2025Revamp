package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.control.PrimePIDConstants;

public class IntakeSubsystem extends SubsystemBase {

  public static class VMap {

    public static final int RollersCanId = 16;
    public static final int NeoLeftCanId = 15;
    public static final int NeoRightCanId = 14;
    public static final boolean RollersInverted = false;
    public static final boolean NeoLeftInverted = false;
    public static final boolean NeoRightInverted = true;
    public static final PrimePIDConstants IntakeAnglePid = new PrimePIDConstants(0.05, 0, 0);
    public static final int PositionDelta = 49;
    public static final int TopLimitSwitchChannel = 4;
    public static final int BottomLimitSwitchChannel = 5;
  }

  private boolean m_angleToggledIn = true;
  private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth); //output
  private PIDController m_anglePid;

  private IIntakeIO m_intakeIO;
private IntakeIOInputs m_inputs = new IntakeIOInputs();
  private IntakeIOOutputs m_outputs = new IntakeIOOutputs();

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public IntakeSubsystem(boolean isReal) {
    setName("Intake");
    if (isReal) {
      m_intakeIO = new IntakeIOReal();
    } else {
      m_intakeIO = new IntakeIOSim();
    }

    SmartDashboard.putNumber("Intake/AngleStartPoint", m_inputs.AngleStartPoint);

    m_anglePid = VMap.IntakeAnglePid.createPIDController(0.02);
    m_anglePid.setSetpoint(m_inputs.AngleStartPoint);
    m_angleToggledIn = true;

    // Set the default command for the subsystem so that it runs the PID loop
    setDefaultCommand(seekAngleSetpointCommand());
  }

  //#region Control Methods

  
  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    m_outputs.rollerMotorOutput = speed;
  }

  /**
   * Sets the speed of the Intake Angle Motors
   * @param speed
   */
  public void setAngleMotorSpeed(double speed) {
    m_outputs.leftMotorOutput = -speed;
    m_outputs.rightMotorOutput = speed;
  }

  /**
   * Sets the Intake Angle to a given position in rotations of the motor shaft
   * @param positionSetpoint
   */
  public void setIntakeRotation() {
    var currentPosition = m_inputs.intakeMotorRightPosition;
    var setpoint = m_angleToggledIn ? m_inputs.AngleStartPoint : (m_inputs.AngleStartPoint - VMap.PositionDelta);
    SmartDashboard.putNumber("Intake/AngleSetpoint", setpoint);

    var pidOutput = m_anglePid.calculate(currentPosition, setpoint);
    SmartDashboard.putNumber("Intake/AnglePIDOutput", pidOutput);

    // artificial limits
    if (currentPosition < m_inputs.AngleStartPoint && pidOutput > 0 && !m_inputs.topLimitSwitch) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 1));
    } else if (
      currentPosition > (m_inputs.AngleStartPoint - VMap.PositionDelta) && pidOutput < 0 && !m_inputs.bottomLimitSwitch
    ) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -1, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  //#endregion

  @Override
  public void periodic() {
    // Level2 Logging
    // SmartDashboard.putBoolean("Intake/ToggledIn", m_angleToggledIn);

    // SmartDashboard.putNumber("Intake/ArmPositionRight", getPositionRight());
    // SmartDashboard.putNumber("Intake/ArmPositionLeft", getPositionLeft());

    // SmartDashboard.putNumber("Intake/RightMotorOutput", m_angleRight.get());
    // SmartDashboard.putNumber("Intake/LeftMotorOutput", m_angleLeft.get());

    // SmartDashboard.putNumber("Intake/RollersOutput", m_rollers.get());
  }

  //#region Commands

  /**
   * Constantly seeks the angle setpoint for the arm. If interrupted, stops the arm motors
   * @return
   */
  public Command seekAngleSetpointCommand() {
    return this.run(() -> setIntakeRotation()).finallyDo(() -> stopArmMotorsCommand());
  }

  /**
   * Sets the rollers to a given speed
   */
  public Command setRollersSpeedCommand(DoubleSupplier speed) {
    return Commands.runOnce(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Runs the rollers at a given speed
   */
  public Command runRollersAtSpeedCommand(DoubleSupplier speed) {
    return Commands.run(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Sets the rollers to eject a note at max speed
   */
  public Command ejectNoteCommand() {
    return Commands.runOnce(() -> runIntakeRollers(-1));
  }

  /**
   * Sets the intake angle to loading position
   */
  public Command setIntakeInCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = true);
  }

  /**
   * Sets the intake angle setpoint to ground position
   */
  public Command setIntakeOutCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = false);
  }

  /**
   * Toggles the intake angle setpoint between in/out
   * @return
   */
  public Command toggleIntakeInAndOutCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = !m_angleToggleDebouncer.calculate(m_angleToggledIn));
  }

  /**
   * Stops the intake arm motors
   * @return
   */
  public Command stopArmMotorsCommand() {
    return Commands.runOnce(() -> {
      m_intakeIO.stopAngleMotors();
    });
  }

  /**
   * Stops the intake rollers
   * @return
   */
  public Command stopRollersCommand() {
    return Commands.runOnce(() -> {
      m_intakeIO.stopRollerMotors();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Set_Intake_Out",
      setIntakeOutCommand(),
      "Set_Intake_In",
      setIntakeInCommand(),
      "Start_Note_Intake",
      setRollersSpeedCommand(() -> 1),
      "Stop_Note_Intake",
      stopRollersCommand(),
      "Eject_Note",
      ejectNoteCommand()
    );
  }
  //#endregion
}
