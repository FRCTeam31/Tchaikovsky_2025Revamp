package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth); //output

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public IntakeSubsystem() {
    setName("Intake");
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

    m_angleStartPoint = getPositionRight();
    SmartDashboard.putNumber("Intake/AngleStartPoint", m_angleStartPoint);

    m_anglePid = VMap.IntakeAnglePid.createPIDController(0.02);
    m_anglePid.setSetpoint(m_angleStartPoint);
    m_angleToggledIn = true;

    // Set the default command for the subsystem so that it runs the PID loop
    setDefaultCommand(seekAngleSetpointCommand());
  }

  //#region Control Methods

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

  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    m_rollers.set(speed);
  }

  /**
   * Sets the speed of the Intake Angle Motors
   * @param speed
   */
  public void setAngleMotorSpeed(double speed) {
    m_angleLeft.set(-speed);
    m_angleRight.set(speed);
  }

  /**
   * Sets the Intake Angle to a given position in rotations of the motor shaft
   * @param positionSetpoint
   */
  public void setIntakeRotation() {
    var currentPosition = getPositionRight();
    var setpoint = m_angleToggledIn ? m_angleStartPoint : (m_angleStartPoint - VMap.PositionDelta);
    SmartDashboard.putNumber("Intake/AngleSetpoint", setpoint);

    var pidOutput = m_anglePid.calculate(currentPosition, setpoint);
    SmartDashboard.putNumber("Intake/AnglePIDOutput", pidOutput);

    // artificial limits
    if (currentPosition < m_angleStartPoint && pidOutput > 0 && !m_topLimitSwitch.get()) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 1));
    } else if (
      currentPosition > (m_angleStartPoint - VMap.PositionDelta) && pidOutput < 0 && !m_bottomLimitSwitch.get()
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
    SmartDashboard.putBoolean("Intake/ToggledIn", m_angleToggledIn);

    SmartDashboard.putNumber("Intake/ArmPositionRight", getPositionRight());
    SmartDashboard.putNumber("Intake/ArmPositionLeft", getPositionLeft());

    SmartDashboard.putNumber("Intake/RightMotorOutput", m_angleRight.get());
    SmartDashboard.putNumber("Intake/LeftMotorOutput", m_angleLeft.get());

    SmartDashboard.putNumber("Intake/RollersOutput", m_rollers.get());
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
      m_angleLeft.stopMotor();
      m_angleRight.stopMotor();
    });
  }

  /**
   * Stops the intake rollers
   * @return
   */
  public Command stopRollersCommand() {
    return Commands.runOnce(() -> {
      m_rollers.stopMotor();
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
