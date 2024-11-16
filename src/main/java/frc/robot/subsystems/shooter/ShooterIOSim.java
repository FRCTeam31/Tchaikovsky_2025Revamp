package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;

@Logged(strategy = Strategy.OPT_IN)
public class ShooterIOSim implements IShooterIO {

    @Override
    public ShooterIOInputs getInputs() {
        return null;
    }

    @Override
    public void setOutputs(ShooterIOOutputs outputs) {

    }

    @Override
    public void StopMotors() {

    }
    
}
