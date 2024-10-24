package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.LEDEffect;

public class SolidPattern extends LEDPattern {

  /**
   * Create a new SolidPattern with a color
   * @param color The color of the pattern
   */
  public SolidPattern(prime.control.LEDs.Color color) {
    super(color, LEDEffect.Solid, 0, false);
  }

  /**
   * Create a new SolidPattern with an RGB color
   * @param r The red value of the color
   * @param g The green value of the color
   * @param b The blue value of the color
   */
  public SolidPattern(int r, int g, int b) {
    super(r, g, b, LEDEffect.Solid, 0, false);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    for (int i = 0; i < length; i++) {
      buffer.setRGB(startingIndex + i, Color.r, Color.g, Color.b);
    }
  }
}
