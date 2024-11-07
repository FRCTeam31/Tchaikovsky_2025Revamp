package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ShooterIOReal implements IShooterIO {

    private TalonFX m_talonFX;
    private VictorSPX m_victorSPX;
    private DoubleSolenoid m_elevationSolenoid;
    private DigitalInput m_noteDetector;

    private IShooterIO shooterIO;
    private ShooterIOInputs shooterInputs = new ShooterIOInputs();
    private ShooterIOOutputs shooterOutputs = new ShooterIOOutputs();

    public ShooterIOReal() {
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
    
    public ShooterIOInputs GetInputs() {
        var inputs = new ShooterIOInputs();

        inputs.noteDetector_state = !m_noteDetector.get();

        return inputs;
    }

    public void SetOutputs(ShooterIOOutputs outputs) {
        var inputs = new ShooterIOInputs();

        outputs.talon_speed = 0;
        outputs.victor_speed = 0;

        m_elevationSolenoid.set(outputs.elevationSolenoid_value);

    }

    @Override
    public void StopMotors() {
        m_talonFX.stopMotor();
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);

        shooterOutputs.talon_speed = 0;
        shooterOutputs.victor_speed = 0;
    }
}
