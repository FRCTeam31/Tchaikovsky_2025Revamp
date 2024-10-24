package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDEffect;

public class BlinkPattern extends LEDPattern {

  private static final double FRAME_COUNT = 2;

  /**
   * Create a new BlinkPattern with a color and speed
   * @param color The color of the pattern
   * @param speed The speed of the pattern in seconds per iteration
   */
  public BlinkPattern(Color color, double speed) {
    super(color, LEDEffect.Blink, speed / FRAME_COUNT, false);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    if (isUpdatable()) {
      Frame = Frame == 0 ? 1 : 0;

      var color = Frame == 0 ? Color : prime.control.LEDs.Color.OFF;

      for (int i = 0; i < length; i++) {
        buffer.setRGB(startingIndex + i, color.r, color.g, color.b);
      }

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
