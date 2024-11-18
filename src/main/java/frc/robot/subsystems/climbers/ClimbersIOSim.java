package frc.robot.subsystems.climbers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;

@Logged(strategy = Strategy.OPT_IN)
public class ClimbersIOSim implements IClimbersIO{
    public DigitalInput leftLimitSwitch;
    public DigitalInput rightLimitSwitch;
    
    @Override
    public ClimbersIOInputs getInputs() {
        return null;
    }

    @Override
    public void setOutputs(ClimbersIOOutputs outputs) {

    }

}
