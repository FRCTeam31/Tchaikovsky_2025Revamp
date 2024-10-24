package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;
import prime.control.LEDs.Patterns.SolidPattern;

public class Shooter extends SubsystemBase {

  public static class VMap {

    public static final int TalonFXCanID = 20;
    public static final int VictorSPXCanID = 19;
    public static final boolean TalonFXInverted = false;
    public static final boolean VictorSPXInverted = false;
    public static final int NoteDetectorDIOChannel = 7;
    public static final int ElevationSolenoidForwardChannel = 6;
    public static final int ElevationSolenoidReverseChannel = 7;
  }

  private PwmLEDs m_leds;
  private TalonFX m_talonFX;
  private VictorSPX m_victorSPX;
  private DoubleSolenoid m_elevationSolenoid;
  private DigitalInput m_noteDetector;

  // #endregion

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public Shooter(PwmLEDs leds) {
    m_leds = leds;
    setName("Shooter");

    m_talonFX = new TalonFX(VMap.TalonFXCanID);
    m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
    m_talonFX.setInverted(true);
    m_talonFX.setNeutralMode(NeutralModeValue.Brake);

    m_victorSPX = new VictorSPX(VMap.VictorSPXCanID);
    m_victorSPX.configFactoryDefault();
    m_victorSPX.setNeutralMode(NeutralMode.Brake);

    m_elevationSolenoid =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        VMap.ElevationSolenoidForwardChannel,
        VMap.ElevationSolenoidReverseChannel
      );

    m_noteDetector = new DigitalInput(VMap.NoteDetectorDIOChannel);
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    m_talonFX.set(speed);
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed * 3);
  }

  public void runGreenWheel(double speed) {
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /**
   * Stops the shooter motors
   */
  public void stopMotors() {
    m_talonFX.stopMotor();
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
    m_leds.restorePersistentStripPattern();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return
   */
  public boolean isNoteLoaded() {
    return !m_noteDetector.get();
  }

  public void setElevator(Value value) {
    m_elevationSolenoid.set(value);
  }

  public void setElevatorUp() {
    setElevator(Value.kForward);
    m_leds.setStripTemporaryPattern(new SolidPattern(Color.WHITE));
  }

  public void setElevatorDown() {
    setElevator(Value.kReverse);
    m_leds.restorePersistentStripPattern();
  }

  //#endregion

  private boolean m_lastNoteDetectedValue = false;

  @Override
  public void periodic() {
    var newNoteDetectedValue = isNoteLoaded();
    if (newNoteDetectedValue != m_lastNoteDetectedValue) {
      if (newNoteDetectedValue && !m_lastNoteDetectedValue) {
        m_leds.setStripTemporaryPattern(new BlinkPattern(prime.control.LEDs.Color.ORANGE, 0.2));
      } else {
        m_leds.restorePersistentStripPattern();
      }

      // Save the new value
      m_lastNoteDetectedValue = newNoteDetectedValue;
    }

    // Level2 Logging
    SmartDashboard.putNumber("Shooter/LaunchMotorOutput", m_talonFX.get());
    SmartDashboard.putNumber("Shooter/LaunchMotorVelocity", m_talonFX.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/GuideMotorOutput", m_victorSPX.getMotorOutputPercent());
    SmartDashboard.putBoolean("Shooter/NoteDetected", newNoteDetectedValue);
  }

  //#region Shooter Commands

  /**
   * Stops the shooter motors
   * @return
   */
  public Command stopMotorsCommand() {
    return Commands.runOnce(() -> stopMotors());
  }

  /**
   * Shootes a note at half speed
   * @return
   */
  public Command scoreInAmpCommand() {
    return Commands.run(() -> runShooter(0.5));
  }

  /**
   * Shootes a note at full speed
   * @return
   */
  public Command startShootingNoteCommand() {
    return Commands.runOnce(() -> {
      runShooter(1);
      m_leds.setStripTemporaryPattern(new ChasePattern(Color.GREEN, 0.25, isNoteLoaded()));
    });
  }

  /**
   * Sets the elevation of the shooter all the way up
   * @return
   */
  public Command setElevationUpCommand() {
    return Commands.runOnce(this::setElevatorUp);
  }

  /**
   * Sets the elevation of the shooter all the way down
   * @return
   */
  public Command setElevationDownCommand() {
    return Commands.runOnce(this::setElevatorDown);
  }

  /**
   * Toggles the elevation of the shooter up/down
   * @return
   */
  public Command toggleElevationCommand() {
    return Commands.runOnce(() -> {
      if (m_elevationSolenoid.get() == Value.kForward) setElevatorDown(); else setElevatorUp();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Set_Elevation_Up",
      setElevationUpCommand(),
      "Set_Elevation_Down",
      setElevationDownCommand(),
      "Start_Shooting",
      startShootingNoteCommand(),
      "Stop_Shooting",
      stopMotorsCommand()
    );
  }
  //#endregion
}
