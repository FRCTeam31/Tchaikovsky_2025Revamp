package frc.robot.subsystems.climbers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
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

@Logged(strategy = Strategy.OPT_IN)
public class ClimbersSubsystem extends SubsystemBase {

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

  private IClimbersIO m_climbIO;
  private ClimbersIOOutputs m_outputs = new ClimbersIOOutputs();
  private ClimbersIOInputs m_Inputs = new ClimbersIOInputs();

  // Member to track if the climb controls are enabled
  private boolean m_climbControlsEnabled = false;

  /**
   * Creates a new Climbers subsystem
   * @param config
   */
  public ClimbersSubsystem(boolean isReal) {
    if (isReal) {
      m_climbIO = new ClimbersIOReal();
    } else {
      m_climbIO = new ClimbersIOSim();
    }
  }

  //#region Control Methods

  /**
     * Raises the desired climber arm
     * @param side The side to raise
     */
    public void raiseArm(Side side) {
        if (side == Side.kLeft && !m_Inputs.leftLimitSwitch) {
          m_outputs.leftMotorOutput = VMap.ClimberUpSpeed;
        }

        if (side == Side.kRight && !m_Inputs.rightLimitSwitch) {
          m_outputs.rightMotorOutput = VMap.ClimberUpSpeed;
        }
    }

    /**
     * Lowers the desired climber arm
     * @param side The side to lower
     * @param speed The speed to lower the arm at
     */ 

    public void lowerArm(Side side, double speed) {
        if (side == Side.kLeft) {
            m_outputs.leftMotorOutput = -speed;
        }

        if (side == Side.kRight) {
          m_outputs.rightMotorOutput = -speed;
        }
    }

    /**
     * Stops the desired climber arm
     * @param side
     */
    public void stopArm(Side side) {
        if (side == Side.kLeft) {
          m_outputs.leftMotorOutput = 0;
            setClutch(Side.kLeft, true);
        }

        if (side == Side.kRight) {
          m_outputs.rightMotorOutput = 0;
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
          m_outputs.ClutchLeft = engaged;

        }

        if (side == Side.kRight) {
          m_outputs.ClutchRight = engaged;

        }
    }
  

  @Override
  public void periodic() {
    m_Inputs = m_climbIO.getInputs();
    m_climbIO.setOutputs(m_outputs);


    DriverDashboard.ClimberControlsActiveBox.setBoolean(m_climbControlsEnabled);

    // Level2 Logging
    
    SmartDashboard.putBoolean("Climbers/ControlsEnabled", m_climbControlsEnabled);
    // outputs
    SmartDashboard.putNumber("Climbers/LeftMotorOutput", m_outputs.leftMotorOutput);
    SmartDashboard.putNumber("Climbers/RightMotorOutput", m_outputs.rightMotorOutput);
    //
    // inputs
    SmartDashboard.putBoolean("Climbers/LeftLimitSwitch", m_Inputs.leftLimitSwitch);
    SmartDashboard.putBoolean("Climbers/RightLimitSwitch", m_Inputs.rightLimitSwitch);
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
          var rightLimitSwitchTriggered = m_Inputs.rightLimitSwitch;
          if (raiseRightArmTriggered && !rightLimitSwitchTriggered) {
            setClutch(Side.kRight, false);
            raiseArm(Side.kRight);
          } else {
            stopArm(Side.kRight);
          }

          // Raise left
          var raiseLeftArmTriggered = raiseLeftArm.getAsBoolean();
          var leftLimitSwitchTriggered = m_Inputs.leftLimitSwitch;
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
      .andThen(new WaitUntilCommand(() -> m_Inputs.leftLimitSwitch || m_Inputs.rightLimitSwitch).withTimeout(2))
      .andThen(() -> {
        stopArm(Side.kLeft);
        stopArm(Side.kRight);
      });
  }
  //#endregion
}
