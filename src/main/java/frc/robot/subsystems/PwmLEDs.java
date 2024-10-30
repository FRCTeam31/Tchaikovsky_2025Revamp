package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PwmLEDs extends SubsystemBase {
    public static class VMap {

        public static final int PwmPort = 9;
        public static final int PixelsPerStrip = 78;
        public static final double BackgroundDimAmount = 0.5;
    }

    private boolean m_isRobotReal;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private byte _loopErrorCounter = 0;

    private ScheduledExecutorService _updateLoopExecutor;
    private LEDPattern m_backgroundPattern = LEDPattern.solid(Color.kGray);
    private LEDPattern m_foregroundPattern = null;

    private Alert m_loopStoppedAlert;
    private Dimensionless m_backgroundDimAmount = Units.Percent.of(50);

    public PwmLEDs(boolean isReal) {
        m_isRobotReal = isReal;
        if (m_isRobotReal) {
            // Initialize the LED strip and buffer
            m_ledBuffer = new AddressableLEDBuffer(VMap.PixelsPerStrip);
            m_led = new AddressableLED(VMap.PwmPort);
            m_led.setLength(m_ledBuffer.getLength());
            m_led.start();

            // Apply a default pattern to the LED strip
            m_backgroundPattern.applyTo(m_ledBuffer);

            // Setup the warning for when the loop stops
            m_loopStoppedAlert = new Alert("[LEDs:ERROR] LED update loop failed.", Alert.AlertType.kWarning);
            m_loopStoppedAlert.set(false);

            // _updateLoopExecutor = Executors.newScheduledThreadPool(1);
            // // Start the pattern update loop at 200Hz with a 3ms delay to running at the same time as periodic functions
            // _updateLoopExecutor.scheduleAtFixedRate(this::updateLedStrip, 3, 5, java.util.concurrent.TimeUnit.MILLISECONDS);
        } else {
            // initialize nothing
        }
    }

    @Override
    public void periodic() {
        if (!m_isRobotReal)
            return;

        /**
         * TODO: Test at 20ms periodic update loop, then test with ScheduledExecutorService. 
         * Determine if the pattern velocity is thrown off, or is simply rendered more smoothly.
         */
        updateLedStrip();
    }

    public Command setBackgroundPattern(LEDPattern backgroundPattern) {
        if (!m_isRobotReal)
            return runOnce(() -> {});

        return runOnce(() -> {
            m_backgroundPattern = backgroundPattern;
        });
    }

    public Command setForegroundPattern(LEDPattern foregroundPattern) {
        if (!m_isRobotReal)
            return runOnce(() -> {});

        return runOnce(() -> {
            m_foregroundPattern = foregroundPattern;
        });
    }

    public Command clearForegroundPattern() {
        if (!m_isRobotReal)
            return runOnce(() -> {});

        return runOnce(() -> {
            m_foregroundPattern = null;
        });
    }

    private void updateLedStrip() {
        if (!m_isRobotReal || _loopErrorCounter > 3)
            return;
            
        try {
            if (m_foregroundPattern == null) {
                m_backgroundPattern.applyTo(m_ledBuffer);
            } else {
                var dimmedBackground = m_backgroundPattern.atBrightness(m_backgroundDimAmount);
                m_foregroundPattern.overlayOn(dimmedBackground).applyTo(m_ledBuffer);
            }

            m_led.setData(m_ledBuffer);
        } catch (Exception e) {
            _loopErrorCounter++;
            DriverStation.reportError("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage(), e.getStackTrace());

            if (_loopErrorCounter > 3) {
                var msg = "[LEDs:ERROR] LED update loop has failed 3 times. Stopping loop.";
                DriverStation.reportError(msg, false);
                System.out.println(msg);
                m_loopStoppedAlert.set(true);
                _updateLoopExecutor.shutdown();
            }
        }
    }
}
