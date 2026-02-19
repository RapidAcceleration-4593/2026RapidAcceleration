package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import java.util.Random;

public class LEDConstants {

    public static final int kPWMChannel = 0;
    public static final int kLEDCount = 150;

    public static enum kColors {
        RED(new Color(255, 0, 0)),
        GREEN(new Color(0, 255, 0)),
        BLUE(new Color(0, 0, 255)),
        YELLOW(new Color(255, 70, 0)),
        ORANGE(new Color(255, 10, 0)),
        PURPLE(new Color(255, 0, 255)),
        BLACK(new Color(0, 0, 0));
        public final Color color;

        kColors(Color color) {
            this.color = color;
        }

        private static final Random random = new Random();

        public static kColors getRandom() {
            kColors[] values = kColors.values();
            return values[random.nextInt(values.length)];
        }
    }

    // General pattern config.
    public static final double kSpeedFactor = 1.0;

    // Gradient trail pattern config.
    public static final int kTrailCount = 3;
    public static final int kTrailSize = 40;

    // Rainbow pattern config.
    public static final double kRainbowFactor = 2.0;
}
