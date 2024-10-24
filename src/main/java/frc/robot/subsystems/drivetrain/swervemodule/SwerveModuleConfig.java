package frc.robot.subsystems.drivetrain.swervemodule;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfig {

  public int DriveMotorCanId;
  public int SteeringMotorCanId;
  public int CANCoderCanId;
  public double CanCoderStartingOffset;
  public Translation2d ModuleLocation;
  public boolean DriveInverted;
  public boolean SteerInverted;

  public ClosedLoopRampsConfigs DriveClosedLoopRampConfiguration = new ClosedLoopRampsConfigs()
    .withTorqueClosedLoopRampPeriod(0.5)
    .withVoltageClosedLoopRampPeriod(0.5)
    .withDutyCycleClosedLoopRampPeriod(0.5);
  public CurrentLimitsConfigs DriveCurrentLimitConfiguration = new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentThreshold(50)
    .withSupplyTimeThreshold(100);

  public SwerveModuleConfig(
    int driveMotorCanId,
    int steeringMotorCanId,
    int canCoderCanId,
    double canCoderStartingOffset,
    boolean driveInverted,
    boolean steerInverted,
    Translation2d location
  ) {
    DriveMotorCanId = driveMotorCanId;
    SteeringMotorCanId = steeringMotorCanId;

    CANCoderCanId = canCoderCanId;
    CanCoderStartingOffset = canCoderStartingOffset;

    DriveInverted = driveInverted;
    SteerInverted = steerInverted;

    ModuleLocation = location;
  }
}
