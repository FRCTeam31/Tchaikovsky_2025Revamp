package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.drivetrain.DrivetrainIOInputs;
import frc.robot.subsystems.drivetrain.DrivetrainIOOutputs;

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
     * Stops the motors in the shooter
     * @implNote Version used in the shooter IO, should only be used when within the shooter subsytem
     * @see ShooterSubsystem stopMotors()
     */
    public void StopMotors();
}
