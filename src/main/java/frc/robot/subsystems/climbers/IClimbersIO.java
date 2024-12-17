package frc.robot.subsystems.climbers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;

@Logged(strategy = Strategy.OPT_IN)
public interface IClimbersIO {


    public ClimbersIOInputs getInputs();
    public void setOutputs(ClimbersIOOutputs outputs);

    
}
