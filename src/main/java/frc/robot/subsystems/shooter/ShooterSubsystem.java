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
import frc.robot.Robot;

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

   @Logged(name = "ShooterIO", importance = Logged.Importance.CRITICAL)
  private IShooterIO m_shooterio = (
    Robot.isReal()
    ? new ShooterIOReal() 
    : new ShooterIOSim()
  );
  private ShooterIOInputs m_inputs;
  @Logged(name = "ShooterIOOutputs", importance = Logged.Importance.CRITICAL)
  private ShooterIOOutputs m_outputs;

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

    m_inputs = m_shooterio.getInputs();
    m_outputs = new ShooterIOOutputs();
    
    m_clearForegroundPatternFunc = restoreLEDPersistentPatternFunc;
    m_setForegroundPatternFunc = setLEDTemporaryPatternFunc;
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    m_outputs.TalonSpeed = speed;
    m_outputs.VictorSpeed = speed * 3;
  }

  public void runGreenWheel(double speed) {
    m_outputs.VictorSpeed = speed;
  }

  /**
   * Stops the shooter motors
   * @implNote Version declared in the ShooterSubsystem.
   * This is the method that, unlike StopMotors(), interacts with the lights on the robot. Because of this, this method will be used almost everywhere.
   * ( StopMotors() in IShooterIO behaves more like multiple lines of code stopping the motors, while stopMotors() in ShooterSubsystem ties it all together. ) 
   * @see IShooterIO StopMotors()
   */
  public void stopMotors() {
    m_shooterio.StopMotors();
    
    m_clearForegroundPatternFunc.run();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return boolean
   */
  public boolean isNoteLoaded() {
    return m_inputs.NoteDetectorState;
  }

  /**
   * Sets the elevator to either kForward or kReverse ( kOff not supported and will instead act as if it is kReverse ).
   * Works by checking if value is kForward, then setting ElevationSolenoidValue to true if they are equal and false if they are not.
   * @implNote A positive ElevationSolenoidValue corresponds to kForward and vice versa
   * @param value
   */
  public void setElevator(Value value) {
    m_outputs.ElevationSolenoidValue = (
      value == Value.kForward
      ? true
      : false
    );
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
    SmartDashboard.putNumber("Shooter/LaunchMotorOutput", m_inputs.TalonState);
    SmartDashboard.putNumber("Shooter/LaunchMotorVelocity", m_inputs.TalonVelocity);
    SmartDashboard.putNumber("Shooter/GuideMotorOutput", m_inputs.VictorOutput);
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
      if (m_inputs.ElevationSolenoidState == true) {
        setElevatorDown();
      } else {
        setElevatorUp();
      }
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
