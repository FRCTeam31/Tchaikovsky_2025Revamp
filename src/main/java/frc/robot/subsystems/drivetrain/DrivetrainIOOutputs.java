package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainIOOutputs {
    public boolean SnapEnabled = false;
    public Rotation2d SnapSetpoint = new Rotation2d();
    public ChassisSpeeds DesiredChassisSpeeds = new ChassisSpeeds();
    public DrivetrainControlMode ControlMode = DrivetrainControlMode.kRobotRelative;
}
