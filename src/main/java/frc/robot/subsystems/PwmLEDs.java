package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import prime.control.LEDs.Patterns.LEDPattern;

public class PwmLEDs extends SubsystemBase {

  public static class VMap {

    public static final int PwmPort = 9;
    public static final int PixelsPerStrip = 78;
  }

  private AddressableLED _led;
  private AddressableLEDBuffer _ledBuffer;

  private final ScheduledExecutorService _updateLoopExecutor = Executors.newScheduledThreadPool(1);
  private LEDPattern _persistentPattern;
  private LEDPattern _temporaryPattern;

  public PwmLEDs() {
    // Initialize the LED strip and buffer
    _ledBuffer = new AddressableLEDBuffer(VMap.PixelsPerStrip);
    _led = new AddressableLED(VMap.PwmPort);
    _led.setLength(_ledBuffer.getLength());

    // Set the strip to a default color and start the LED strip
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, 100, 100, 100);
    }
    _led.setData(_ledBuffer);
    _led.start();

    // Start the pattern update loop at 142hz with a default pattern
    _updateLoopExecutor.scheduleAtFixedRate(this::ledUpdateLoop, 0, 7, java.util.concurrent.TimeUnit.MILLISECONDS);
  }

  /**
   * Set the persistent pattern of the LED strip
   */
  public void setStripPersistentPattern(LEDPattern pattern) {
    _persistentPattern = pattern;
  }

  /**
   * Set the temporary pattern of the LED strip
   */
  public void setStripTemporaryPattern(LEDPattern pattern) {
    _temporaryPattern = pattern;
  }

  /**
   * Set the LED strip back to it's persistent pattern
   */
  public void restorePersistentStripPattern() {
    _temporaryPattern = null;
  }

  private byte _loopErrorCounter = 0;

  private void ledUpdateLoop() {
    try {
      // If the temporary pattern for this section is not null, use it instead of the persistent pattern
      var pattern = _temporaryPattern != null ? _temporaryPattern : _persistentPattern;

      // If the pattern is not null, update the LED strip
      if (pattern != null) {
        // Request for the pattern to calculate the next frame and update the buffer
        pattern.updateBuffer(0, VMap.PixelsPerStrip, _ledBuffer);

        // Update the LED strip with the new buffer
        _led.setData(_ledBuffer);
      }
    } catch (Exception e) {
      _loopErrorCounter++;
      DriverStation.reportError("[LEDs:ERROR] Error in update loop: " + e.getMessage(), e.getStackTrace());

      if (_loopErrorCounter > 3) {
        var msg = "[LEDs:ERROR] LED update loop has failed 3 times. Stopping loop.";
        DriverStation.reportError(msg, false);
        System.out.println(msg);
        _updateLoopExecutor.shutdown();
      }
    }
  }
}
