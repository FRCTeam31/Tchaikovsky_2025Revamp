package frc.robot.subsystems.drivetrain.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.maps.*;
import prime.control.PrimePIDConstants;
import prime.util.CTREConverter;

public class SwerveModuleIOReal implements ISwerveModuleIO {

  private SwerveModuleMap m_map;
  private SwerveModuleIOInputs m_inputs;

  // Devices
  private CANSparkMax m_SteeringMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_encoder;
  private PIDController m_steeringPidController;

  // CTRE Velocity/volts descriptor.
  // Starts at velocity 0, no feed forward. Uses PID slot 0.
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false, false);

  public SwerveModuleIOReal(SwerveModuleMap moduleMap) {
    m_map = moduleMap;

    setupSteeringMotor(DriveMap.SteeringPID);
    setupDriveMotor(DriveMap.DrivePID);
    setupCanCoder();
  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    var rotation = Rotation2d.fromRotations(m_encoder.getPosition().getValueAsDouble());
    var speedMps = CTREConverter.rotationsToMeters(
      m_driveMotor.getVelocity().getValueAsDouble(),
      DriveMap.DriveWheelCircumferenceMeters,
      DriveMap.DriveGearRatio
    );
    var distanceMeters = CTREConverter.rotationsToMeters(
      m_driveMotor.getPosition().getValueAsDouble(),
      DriveMap.DriveWheelCircumferenceMeters,
      DriveMap.DriveGearRatio
    );

    m_inputs.ModuleState.angle = rotation;
    m_inputs.ModuleState.speedMetersPerSecond = speedMps;
    m_inputs.ModulePosition.angle = rotation;
    m_inputs.ModulePosition.distanceMeters = distanceMeters;

    return m_inputs;
  }

  @Override
  public void setOutputs(SwerveModuleIOOutputs outputs) {
    setDesiredState(outputs.DesiredState);
  }

  @Override
  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_SteeringMotor.stopMotor();
  }

  /**
   * Configures the steering motor and PID controller
   */
  private void setupSteeringMotor(PrimePIDConstants pid) {
    m_SteeringMotor = new CANSparkMax(m_map.SteeringMotorCanId, MotorType.kBrushless);
    m_SteeringMotor.restoreFactoryDefaults();

    m_SteeringMotor.setSmartCurrentLimit(100, 80);
    m_SteeringMotor.clearFaults();
    m_SteeringMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_SteeringMotor.setInverted(m_map.SteerInverted); // CCW inversion

    // Create a PID controller to calculate steering motor output
    m_steeringPidController = pid.createPIDController(0.02);
    m_steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
    m_steeringPidController.setTolerance((1 / 360.0) * 2); // 2 degrees in units of rotations
  }

  /**
   * Configures the drive motors
   * @param pid
   */
  private void setupDriveMotor(PrimePIDConstants pid) {
    m_driveMotor = new TalonFX(m_map.DriveMotorCanId);
    m_driveMotor.clearStickyFaults();
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration()); // Reset to factory default

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    // Set the PID values for slot 0
    driveMotorConfig.Slot0 =
      new Slot0Configs().withKP(pid.kP).withKI(pid.kI).withKD(pid.kD).withKS(pid.kS).withKV(pid.kV);

    // Set the voltage limits
    driveMotorConfig.Voltage.PeakForwardVoltage = 12;
    driveMotorConfig.Voltage.PeakReverseVoltage = -12;

    // Set the current limits
    driveMotorConfig.withCurrentLimits(m_map.DriveCurrentLimitConfiguration);

    // Set the ramp rates
    driveMotorConfig.withClosedLoopRamps(m_map.DriveClosedLoopRampConfiguration);
    driveMotorConfig.MotorOutput.Inverted = m_map.DriveInverted 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive; // Clockwise Inversion

    // Apply the configuration
    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Configures the CANCoder
   */
  private void setupCanCoder() {
    m_encoder = new CANcoder(m_map.CANCoderCanId);
    m_encoder.clearStickyFaults();
    m_encoder.getConfigurator().apply(new CANcoderConfiguration());

    // AbsoluteSensorRangeValue
    m_encoder
      .getConfigurator()
      .apply(
        new CANcoderConfiguration()
          .withMagnetSensor(
            new MagnetSensorConfigs()
              .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
              .withMagnetOffset(-m_map.CanCoderStartingOffset)
          )
      );
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The optimized state of the module that we'd like to be at in this
   *                     period
   */
  private void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Set the drive motor to the desired speed
    var speedRotationsPerSecond = CTREConverter.metersToRotations(
      desiredState.speedMetersPerSecond,
      DriveMap.DriveWheelCircumferenceMeters,
      DriveMap.DriveGearRatio
    );

    m_driveMotor.setControl(m_voltageVelocity.withVelocity(speedRotationsPerSecond));

    // Set the steering motor to the desired angle
    var setpoint = desiredState.angle.getRotations() % 1;
    if (setpoint < 0) setpoint += 1;

    var newOutput = m_steeringPidController.calculate(m_inputs.ModuleState.angle.getRotations(), setpoint);

    m_SteeringMotor.set(MathUtil.clamp(newOutput, -1, 1));
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the shortest path to drive at the desired angle
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    Rotation2d currentAngle = m_inputs.ModulePosition.angle;
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))
      );
    } else {
      return desiredState;
    }
  }
}
