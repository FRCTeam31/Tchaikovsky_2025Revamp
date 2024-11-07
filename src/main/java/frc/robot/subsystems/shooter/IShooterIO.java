package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IShooterIO {

    public static class ShooterIOInputs {

        public boolean noteDetector_state;

    }
    
    public static class ShooterIOOutputs {

        public double victor_speed;
        public double talon_speed;

        public Value elevationSolenoid_value;

    }

    public void StopMotors();
}
