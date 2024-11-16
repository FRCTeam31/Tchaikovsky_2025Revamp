package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;

@Logged(strategy = Strategy.OPT_IN)
public interface IShooterIO {

    /**
     * Gets the inputs from the robot to be used in the code
     * @return ShooterIOInputs
     */
    @Logged(name = "Inputs", importance = Logged.Importance.CRITICAL)
    public ShooterIOInputs getInputs();

    /**
     * Sets the outputs given by the code to be used in the robot
     * @param outputs
     */
    public void setOutputs(ShooterIOOutputs outputs);

    /**
     * Directly interacts with IShooterIO, stopping the motors in the shooter
     * @implNote Version declared in the Shooter IO.
     * This method is used to directly communicate to the interface and whichever class is currently running (real or sim).
     * This should mainly only be called from within methods in the ShooterSubsytem class.
     * @see ShooterSubsystem stopMotors()
     */
    public void StopMotors();
}
