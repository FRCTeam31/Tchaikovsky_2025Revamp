package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@Logged(strategy = Strategy.OPT_IN)
public class ShooterIOReal implements IShooterIO {

    private TalonFX m_talonFX;
    private VictorSPX m_victorSPX;
    private DoubleSolenoid m_elevationSolenoid;
    private DigitalInput m_noteDetector;

    private ShooterIOOutputs m_outputs;

    public ShooterIOReal() {
        m_outputs = new ShooterIOOutputs();

        m_talonFX = new TalonFX(ShooterSubsystem.VMap.TalonFXCanID);
        var talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_talonFX.getConfigurator().apply(talonConfig);
        m_talonFX.setNeutralMode(NeutralModeValue.Brake);

        m_victorSPX = new VictorSPX(ShooterSubsystem.VMap.VictorSPXCanID);
        m_victorSPX.configFactoryDefault();
        m_victorSPX.setNeutralMode(NeutralMode.Brake);

        m_elevationSolenoid =
          new DoubleSolenoid(
            30,
            PneumaticsModuleType.REVPH,
            ShooterSubsystem.VMap.ElevationSolenoidForwardChannel,
            ShooterSubsystem.VMap.ElevationSolenoidReverseChannel
          );

        m_noteDetector = new DigitalInput(ShooterSubsystem.VMap.NoteDetectorDIOChannel);
    }

    //#region IO Methods

    @Override
    public ShooterIOInputs getInputs() {
        var inputs = new ShooterIOInputs();

        inputs.TalonState = m_talonFX.get();
        inputs.TalonVelocity = m_talonFX.getVelocity().getValueAsDouble();

        inputs.VictorOutput = m_victorSPX.getMotorOutputPercent();

        inputs.ElevationSolenoidState = (
            m_elevationSolenoid.get() == Value.kForward
            ? true
            : false
        );

        inputs.NoteDetectorState = !m_noteDetector.get();

        return inputs;
    }

    @Override
    public void setOutputs(ShooterIOOutputs outputs) {

        m_talonFX.set(outputs.TalonSpeed);
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, outputs.VictorSpeed);

        m_elevationSolenoid.set(
            outputs.ElevationSolenoidValue
            ? Value.kForward
            : Value.kReverse
        );
    }

    @Override
    public void StopMotors() {

        m_talonFX.stopMotor();
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);

        m_outputs.TalonSpeed = 0;
        m_outputs.VictorSpeed = 0;
    }
    
    //#endregion
}
