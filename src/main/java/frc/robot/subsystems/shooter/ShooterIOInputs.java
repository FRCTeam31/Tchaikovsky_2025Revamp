package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;


/**
 * The inputs used by the shooter subsystem
 */
@Logged
public class ShooterIOInputs {

        public double TalonState;
        public double TalonVelocity;
        public double VictorOutput;

        public boolean ElevationSolenoidState;

        public boolean NoteDetectorState;

}