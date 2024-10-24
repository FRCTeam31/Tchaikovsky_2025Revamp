package prime.control.LEDs;

import edu.wpi.first.wpilibj.DriverStation;

public class Color {

  public int r;
  public int g;
  public int b;

  public Color(int r, int g, int b) {
    if (r < 0 || g < 0 || b < 0 || r > 255 || g > 255 || b > 255) {
      DriverStation.reportError("Invalid color", false);
      return;
    }

    this.r = r;
    this.g = g;
    this.b = b;
  }

  // Predefined static colors
  public static final Color OFF = new Color(0, 0, 0);

  public static final Color RED = new Color(255, 0, 0);

  public static final Color GREEN = new Color(0, 255, 0);

  public static final Color BLUE = new Color(0, 0, 255);

  public static final Color WHITE = new Color(255, 255, 255);

  public static final Color YELLOW = new Color(255, 255, 0);

  public static final Color CYAN = new Color(0, 255, 255);

  public static final Color MAGENTA = new Color(255, 0, 255);

  public static final Color ORANGE = new Color(255, 165, 0);

  public static final Color PINK = new Color(255, 192, 203);

  public static final Color PURPLE = new Color(128, 0, 128);

  public static final Color LIME = new Color(0, 255, 0);

  public static final Color TEAL = new Color(0, 128, 128);

  public static final Color AQUA = new Color(0, 255, 255);

  public static final Color VIOLET = new Color(238, 130, 238);

  public static final Color BROWN = new Color(165, 42, 42);

  public static final Color TAN = new Color(210, 180, 140);

  public static final Color BEIGE = new Color(245, 245, 220);

  public static final Color MAROON = new Color(128, 0, 0);

  public static final Color OLIVE = new Color(128, 128, 0);

  public static final Color NAVY = new Color(0, 0, 128);
}
