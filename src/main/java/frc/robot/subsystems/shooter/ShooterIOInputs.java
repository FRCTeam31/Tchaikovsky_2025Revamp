package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


/**
 * The inputs used by the shooter subsystem
 */
@Logged
public class ShooterIOInputs {

        public double talon_state;
        public double talon_velocity;
        public double victor_output;

        public Value elevationSolenoid_state;

        public boolean noteDetector_state;

}