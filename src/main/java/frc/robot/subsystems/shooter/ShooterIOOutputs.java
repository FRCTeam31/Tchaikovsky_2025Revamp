package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


/**
 * The outputs used by the shooter subsytem
 */
@Logged
public class ShooterIOOutputs {

        public double victor_speed;
        public double talon_speed;

        public Value elevationSolenoid_value;

}
