package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sim.MotorSim;
import frc.robot.subsystems.drivetrain.DriveMap;

public class SwerveModuleIOSim implements ISwerveModuleIO {

  private SwerveModuleIOInputs m_inputs;

  // Devices
  private MotorSim m_driveMotor;
  private double _steerAngle = 0;

  public SwerveModuleIOSim(SwerveModuleConfig moduleMap) {
    m_driveMotor = new MotorSim(DriveMap.MaxSpeedMetersPerSecond);
  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    m_inputs.ModuleState.angle = Rotation2d.fromDegrees(_steerAngle);
    m_inputs.ModuleState.speedMetersPerSecond = m_driveMotor.getVelocity();
    m_inputs.ModulePosition.angle = Rotation2d.fromDegrees(_steerAngle);
    m_inputs.ModulePosition.distanceMeters = m_driveMotor.getDistance();

    return m_inputs;
  }

  @Override
  public void setOutputs(SwerveModuleIOOutputs outputs) {
    _steerAngle = outputs.DesiredState.angle.getDegrees();
    var normalizedSpeed = outputs.DesiredState.speedMetersPerSecond / DriveMap.MaxSpeedMetersPerSecond;
    m_driveMotor.set(normalizedSpeed);
  }

  @Override
  public void stopMotors() {
    m_driveMotor.stopMotor();
  }
}
