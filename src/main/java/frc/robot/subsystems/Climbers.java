package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DriverDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase {

  public static class VMap {

    public static final int VictorSPXLeftCanID = 18;
    public static final int VictorSPXRightCanID = 17;
    public static final boolean LeftInverted = true;
    public static final boolean RightInverted = true;
    public static final double ClimberUpSpeed = 0.5;
    public static final int ClimberDownSpeed = -1;
    public static final int LeftLimitSwitchDIOChannel = 2;
    public static final int RightLimitSwitchDIOChannel = 3;
    public static final int LeftSolenoidForwardChannel = 8;
    public static final int LeftSolenoidReverseChannel = 9;
    public static final int RightSolenoidForwardChannel = 10;
    public static final int RightSolenoidReverseChannel = 11;
  }

  public enum Side {
    kLeft,
    kRight,
  }

  // Motors
  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;

  // Limit Switches
  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;

  // Clutch Solenoids
  private DoubleSolenoid m_clutchSolenoidLeft;
  private DoubleSolenoid m_clutchSolenoidRight;

  // Member to track if the climb controls are enabled
  private boolean m_climbControlsEnabled = false;

  /**
   * Creates a new Climbers subsystem
   * @param config
   */
  public Climbers() {
    m_leftVictorSPX = new VictorSPX(VMap.VictorSPXLeftCanID);
    m_leftVictorSPX.configFactoryDefault();
    m_leftVictorSPX.setInverted(VMap.LeftInverted);
    m_leftVictorSPX.setNeutralMode(NeutralMode.Brake);
    m_leftVictorSPX.configOpenloopRamp(0.5);

    m_rightVictorSPX = new VictorSPX(VMap.VictorSPXRightCanID);
    m_rightVictorSPX.configFactoryDefault();
    m_rightVictorSPX.setInverted(VMap.RightInverted);
    m_rightVictorSPX.setNeutralMode(NeutralMode.Brake);
    m_rightVictorSPX.configOpenloopRamp(0.5);

    m_leftLimitSwitch = new DigitalInput(VMap.LeftLimitSwitchDIOChannel);
    m_rightLimitSwitch = new DigitalInput(VMap.RightLimitSwitchDIOChannel);

    m_clutchSolenoidLeft =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        VMap.LeftSolenoidForwardChannel,
        VMap.LeftSolenoidReverseChannel
      );
    m_clutchSolenoidRight =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        VMap.RightSolenoidForwardChannel,
        VMap.RightSolenoidReverseChannel
      );
  }

  //#region Control Methods

  /**
   * Raises the desired climber arm
   * @param side The side to raise
   */
  public void raiseArm(Side side) {
    if (side == Side.kLeft && !m_leftLimitSwitch.get()) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, VMap.ClimberUpSpeed);
    }

    if (side == Side.kRight && !m_rightLimitSwitch.get()) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, VMap.ClimberUpSpeed);
    }
  }

  /**
   * Lowers the desired climber arm
   * @param side The side to lower
   * @param speed The speed to lower the arm at
   */
  public void lowerArm(Side side, double speed) {
    if (side == Side.kLeft) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
    }

    if (side == Side.kRight) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
    }
  }

  /**
   * Stops the desired climber arm
   * @param side
   */
  public void stopArm(Side side) {
    if (side == Side.kLeft) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
      setClutch(Side.kLeft, true);
    }

    if (side == Side.kRight) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
      setClutch(Side.kRight, true);
    }
  }

  /**
   * Engages / disengages the desired clutch
   * @param side The side to engage / disengage
   * @param engaged Whether to engage or disengage the clutch
   */
  public void setClutch(Side side, boolean engaged) {
    if (side == Side.kLeft) {
      m_clutchSolenoidLeft.set(engaged ? Value.kForward : Value.kReverse);
    }

    if (side == Side.kRight) {
      m_clutchSolenoidRight.set(engaged ? Value.kForward : Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // d_leftLimitEntry.setBoolean(m_leftLimitSwitch.get());
    // d_rightLimitEntry.setBoolean(m_rightLimitSwitch.get());
    DriverDashboard.ClimberControlsActiveBox.setBoolean(m_climbControlsEnabled);

    // Level2 Logging
    SmartDashboard.putBoolean("Climbers/ControlsEnabled", m_climbControlsEnabled);
    SmartDashboard.putNumber("Climbers/LeftMotorOutput", m_leftVictorSPX.getMotorOutputPercent());
    SmartDashboard.putNumber("Climbers/RightMotorOutput", m_rightVictorSPX.getMotorOutputPercent());
    SmartDashboard.putBoolean("Climbers/LeftLimitSwitch", m_leftLimitSwitch.get());
    SmartDashboard.putBoolean("Climbers/RightLimitSwitch", m_rightLimitSwitch.get());
  }

  //#endregion

  //#region Commands

  /**
   * Toggles the Climbers Controls
   */
  public Command toggleClimbControlsCommand() {
    return Commands.runOnce(() -> {
      m_climbControlsEnabled = !m_climbControlsEnabled;
    });
  }

  /**
   * Continually raises / lowers the two arms based on controller inputs
   */
  public Command defaultClimbingCommand(
    BooleanSupplier raiseRightArm,
    BooleanSupplier raiseLeftArm,
    DoubleSupplier lowerRightArm,
    DoubleSupplier lowerLeftArm
  ) {
    return this.run(() -> {
        // Raise only if climbing controls is enabled
        if (m_climbControlsEnabled) {
          // Raise Right
          var raiseRightArmTriggered = raiseRightArm.getAsBoolean();
          var rightLimitSwitchTriggered = m_rightLimitSwitch.get();
          if (raiseRightArmTriggered && !rightLimitSwitchTriggered) {
            setClutch(Side.kRight, false);
            raiseArm(Side.kRight);
          } else {
            stopArm(Side.kRight);
          }

          // Raise left
          var raiseLeftArmTriggered = raiseLeftArm.getAsBoolean();
          var leftLimitSwitchTriggered = m_leftLimitSwitch.get();
          if (raiseLeftArmTriggered && !leftLimitSwitchTriggered) {
            setClutch(Side.kLeft, false);
            raiseArm(Side.kLeft);
          } else {
            stopArm(Side.kLeft);
          }

          // Lower Right
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Side.kRight, true);
            lowerArm(Side.kRight, MathUtil.applyDeadband(lowerRightArm.getAsDouble(), 0.1));
          }

          // Lower left
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Side.kLeft, true);
            lowerArm(Side.kLeft, MathUtil.applyDeadband(lowerLeftArm.getAsDouble(), 0.1));
          }
        }
      });
  }

  /**
   * Sequentially disengages the clutches and raises the arms until both limit switches have been hit.
   */
  public SequentialCommandGroup setArmsUpCommand() {
    return this.runOnce(() -> {
        setClutch(Side.kLeft, false);
        setClutch(Side.kRight, false);
      })
      .andThen(Commands.waitSeconds(0.075))
      .andThen(
        this.runOnce(() -> {
            raiseArm(Side.kLeft);
            raiseArm(Side.kRight);
          })
      )
      .andThen(new WaitUntilCommand(() -> m_leftLimitSwitch.get() || m_rightLimitSwitch.get()).withTimeout(2))
      .andThen(() -> {
        stopArm(Side.kLeft);
        stopArm(Side.kRight);
      });
  }
  //#endregion
}
