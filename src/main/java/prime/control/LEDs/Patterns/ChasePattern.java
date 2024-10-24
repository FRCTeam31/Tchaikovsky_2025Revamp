package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.LEDEffect;

public class ChasePattern extends LEDPattern {

  private static final int CHASE_LENGTH = 8; // Length of the chasing group
  private static final int FADE_LENGTH = 16; // Length of the fading tail

  /**
   * Create a new ChasePattern with a color and speed
   *
   * @param color The color of the pattern
   * @param chaseSpeedSeconds The speed of the pattern in seconds per iteration
   */
  public ChasePattern(prime.control.LEDs.Color color, double chaseSpeedSeconds, boolean reversed) {
    super(color, LEDEffect.Chase, chaseSpeedSeconds, reversed);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    var totalFrameCount = length + CHASE_LENGTH + FADE_LENGTH;
    var frameSpeedS = EffectSpeedSeconds / totalFrameCount;
    var currentTime = System.currentTimeMillis();

    if (currentTime - LastFrameTime >= (frameSpeedS * 1000)) {
      if (!Reversed) {
        // Set the chase group
        var chaseGroupFirst = startingIndex + Frame;
        var chaseGroupLast = chaseGroupFirst - CHASE_LENGTH;

        for (int i = startingIndex; i < (startingIndex + length); i++) {
          // If i is a chase pixel, set it to ON
          if (i > chaseGroupLast && i <= chaseGroupFirst) {
            buffer.setRGB((int) i, Color.r, Color.g, Color.b);
          }

          // If it's within FADE_LENGTH of the last chase pixel, fade it out
          var lastFade = chaseGroupLast - FADE_LENGTH;
          if (i <= chaseGroupLast && i > lastFade) {
            var distanceFromLastChase = Math.abs((chaseGroupLast + 1) - i);
            double brightnessScalar = (1d / (double) FADE_LENGTH) * (double) (FADE_LENGTH - distanceFromLastChase);

            var scaledColor = new prime.control.LEDs.Color(
              (int) (Color.r * brightnessScalar),
              (int) (Color.g * brightnessScalar),
              (int) (Color.b * brightnessScalar)
            );

            buffer.setRGB(i, scaledColor.r, scaledColor.g, scaledColor.b);
          }

          if (i <= lastFade) {
            buffer.setRGB(i, 0, 0, 0);
          }
        }

        Frame = Frame + 1;
        if (Frame >= totalFrameCount) {
          Frame = 0;
        }
      } else {
        // Set the chase group
        var chaseGroupFirst = startingIndex + length - Frame;
        var chaseGroupLast = chaseGroupFirst + CHASE_LENGTH;
        for (long i = chaseGroupLast; i >= chaseGroupFirst; i--) {
          if (i >= startingIndex && i < startingIndex + length) {
            buffer.setRGB((int) i, Color.r, Color.g, Color.b);
          }
        }

        // Set the fading trail
        var fadeTrailFirst = chaseGroupLast + 1;
        var fadeTrailLast = fadeTrailFirst + FADE_LENGTH;
        for (long i = fadeTrailLast; i >= fadeTrailFirst; i--) {
          if (i >= startingIndex && i < startingIndex + length) {
            var brightnessScalar = (float) (length - i) / FADE_LENGTH;
            buffer.setRGB(
              (int) i,
              (byte) (Color.r * brightnessScalar),
              (byte) (Color.g * brightnessScalar),
              (byte) (Color.b * brightnessScalar)
            );
          }
        }

        Frame = Frame - 1;
        if (Frame <= 0) {
          Frame = totalFrameCount;
        }
      }

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
