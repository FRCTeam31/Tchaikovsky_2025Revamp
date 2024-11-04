package frc.robot.maps;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class SwerveModuleMap {

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
    .withSupplyCurrentLimit(Current.ofBaseUnits(40, Units.Amp));

  public SwerveModuleMap(
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
