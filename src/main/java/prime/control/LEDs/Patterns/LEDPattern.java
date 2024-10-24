package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDEffect;

public abstract class LEDPattern {

  protected static final double MIN_FRAME_SPEED = 0.007;

  /**
   * The color of the pattern
   */
  public Color Color;

  /**
   * The effect of the pattern
   */
  public LEDEffect Effect;

  /**
   * The speed of the pattern in seconds per iteration
   */
  public double EffectSpeedSeconds;

  /**
   * Whether the pattern is reversed. This is sometimes controlled by the pattern itself
   */
  public boolean Reversed;

  // State variables
  protected int Frame = 0;
  protected long LastFrameTime = 0;

  public LEDPattern(int r, int g, int b, LEDEffect effect, double effectSpeedSeconds, boolean reversed) {
    this.Color = new Color((byte) r, (byte) g, (byte) b);
    this.Effect = effect;
    this.Reversed = reversed;

    if (effectSpeedSeconds < MIN_FRAME_SPEED) {
      System.out.println("Speed for LED effect " + effect.name() + " is too fast, setting to minimum speed of 7ms");
      this.EffectSpeedSeconds = MIN_FRAME_SPEED;
    } else {
      this.EffectSpeedSeconds = effectSpeedSeconds;
    }
  }

  public LEDPattern(Color color, LEDEffect effect, double effectSpeedSeconds, boolean reversed) {
    this.Color = color;
    this.Effect = effect;
    this.Reversed = reversed;

    if (effectSpeedSeconds < MIN_FRAME_SPEED) {
      System.out.println("Speed for LED effect " + effect.name() + " is too fast, setting to minimum speed of 7ms");
      this.EffectSpeedSeconds = MIN_FRAME_SPEED;
    } else {
      this.EffectSpeedSeconds = effectSpeedSeconds;
    }
  }

  /**
   * Update the buffer with the pattern
   * @param startingIndex The starting index of the buffer to update
   * @param length The length of the buffer to update
   * @param buffer The buffer to update
   */
  public abstract void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer);

  /**
   * Check if the pattern is the same as another pattern
   * @param other The other pattern to compare to
   */
  public boolean isSameAs(LEDPattern other) {
    return (
      this.Color.equals(other.Color) &&
      this.Effect == other.Effect &&
      this.EffectSpeedSeconds == other.EffectSpeedSeconds &&
      this.Reversed == other.Reversed
    );
  }

  /**
   * Check if the pattern is updatable based on the speed
   */
  protected boolean isUpdatable() {
    var currentTime = System.currentTimeMillis();

    return currentTime - LastFrameTime >= (EffectSpeedSeconds * 1000);
  }
}
