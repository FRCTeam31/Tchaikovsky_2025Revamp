package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class SwerveModuleIOInputs {
    public SwerveModulePosition ModulePosition = new SwerveModulePosition();
    public SwerveModuleState ModuleState = new SwerveModuleState();
}