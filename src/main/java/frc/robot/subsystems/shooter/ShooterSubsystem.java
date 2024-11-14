package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.Consumer;

@Logged(strategy = Strategy.OPT_IN)
public class ShooterSubsystem extends SubsystemBase {

  public static class VMap {

    public static final int TalonFXCanID = 20;
    public static final int VictorSPXCanID = 19;
    public static final boolean TalonFXInverted = false;
    public static final boolean VictorSPXInverted = false;
    public static final int NoteDetectorDIOChannel = 7;
    public static final int ElevationSolenoidForwardChannel = 6;
    public static final int ElevationSolenoidReverseChannel = 7;
  }

  private Runnable m_clearForegroundPatternFunc;
  private Consumer<LEDPattern> m_setForegroundPatternFunc;
  private LEDPattern m_elevatorUpLEDPattern = LEDPattern.solid(Color.kWhite);
  private LEDPattern m_shootingLEDPattern = LEDPattern.steps(Map.of(0.0, Color.kGreen, 0.10, Color.kBlack))
    .scrollAtRelativeSpeed(Units.Hertz.of(4));
  private LEDPattern m_noteDetectedLEDPattern = LEDPattern.solid(Color.kOrange)
    .blink(Units.Seconds.of(0.2));

  private IShooterIO shooterIO;
  private ShooterIOInputs shooterInputs = new ShooterIOInputs();
  private ShooterIOOutputs shooterOutputs = new ShooterIOOutputs();

  // #endregion

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public ShooterSubsystem(
    Runnable restoreLEDPersistentPatternFunc,
    Consumer<LEDPattern> setLEDTemporaryPatternFunc
  ) {
    setName("Shooter");
    
    m_clearForegroundPatternFunc = restoreLEDPersistentPatternFunc;
    m_setForegroundPatternFunc = setLEDTemporaryPatternFunc;
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    shooterOutputs.talon_speed = speed;
    shooterOutputs.victor_speed = speed * 3;
  }

  public void runGreenWheel(double speed) {
    shooterOutputs.victor_speed = speed;
  }

  /**
   * Stops the shooter motors
   * @implNote Version declared in the subsytem, should be used when interacting with shooter subsytem from outside
   * @see IShooterIO StopMotors()
   */
  public void stopMotors() {
    shooterIO.StopMotors();
    
    m_clearForegroundPatternFunc.run();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return boolean
   */
  public boolean isNoteLoaded() {
    // return !m_noteDetector.get();
    return shooterInputs.noteDetector_state;
  }

  public void setElevator(Value value) {
    // m_elevationSolenoid.set(value);
    shooterOutputs.elevationSolenoid_value = value;
  }

  public void setElevatorUp() {
    setElevator(Value.kForward);
    m_setForegroundPatternFunc.accept(m_elevatorUpLEDPattern);
  }

  public void setElevatorDown() {
    setElevator(Value.kReverse);
    m_clearForegroundPatternFunc.run();
  }

  //#endregion

  private boolean m_lastNoteDetectedValue = false;

  @Override
  public void periodic() {
    var newNoteDetectedValue = isNoteLoaded();
    if (newNoteDetectedValue != m_lastNoteDetectedValue) {
      if (newNoteDetectedValue && !m_lastNoteDetectedValue) {
        m_setForegroundPatternFunc.accept(m_noteDetectedLEDPattern);
      } else {
        m_clearForegroundPatternFunc.run();
      }

      // Save the new value
      m_lastNoteDetectedValue = newNoteDetectedValue;
    }

    // Level2 Logging
    SmartDashboard.putNumber("Shooter/LaunchMotorOutput", shooterInputs.talon_state);
    SmartDashboard.putNumber("Shooter/LaunchMotorVelocity", shooterInputs.talon_velocity);
    SmartDashboard.putNumber("Shooter/GuideMotorOutput", shooterInputs.victor_output);
    SmartDashboard.putBoolean("Shooter/NoteDetected", newNoteDetectedValue);
  }

  //#region Shooter Commands

  /**
   * Stops the shooter motors
   * @return Command
   */
  public Command stopMotorsCommand() {
    return Commands.runOnce(() -> stopMotors());
  }

  /**
   * Shootes a note at half speed
   * @return Command
   */
  public Command scoreInAmpCommand() {
    return Commands.run(() -> runShooter(0.5));
  }

  /**
   * Shootes a note at full speed
   * @return Command
   */
  public Command startShootingNoteCommand() {
    return Commands.runOnce(() -> {
      runShooter(1);
      m_setForegroundPatternFunc.accept(m_shootingLEDPattern);
    });
  }

  /**
   * Sets the elevation of the shooter all the way up
   * @return Command
   */
  public Command setElevationUpCommand() {
    return Commands.runOnce(this::setElevatorUp);
  }

  /**
   * Sets the elevation of the shooter all the way down
   * @return Command
   */
  public Command setElevationDownCommand() {
    return Commands.runOnce(this::setElevatorDown);
  }

  /**
   * Toggles the elevation of the shooter up/down
   * @return Command
   */
  public Command toggleElevationCommand() {
    return Commands.runOnce(() -> {
      if (shooterInputs.elevationSolenoid_state == Value.kForward) setElevatorDown(); else setElevatorUp();
    });
  }

  /**
   * Links named commands used in PathPlanner to methods in the subsystem
   */
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
