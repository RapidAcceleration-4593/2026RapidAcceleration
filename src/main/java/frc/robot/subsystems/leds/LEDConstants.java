package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import java.util.concurrent.ThreadLocalRandom;

public final class LEDConstants {

    public static final int kPWMChannel = 0;
    public static final int kLEDCount = 150;

    public static enum kColors {
        RED(new Color(255, 0, 0)),
        GREEN(new Color(0, 255, 0)),
        BLUE(new Color(0, 0, 255)),
        YELLOW(new Color(255, 70, 0)),
        ORANGE(new Color(255, 30, 0)),
        PURPLE(new Color(255, 0, 255));
        public final Color color;

        kColors(Color color) {
            this.color = color;
        }

        public static kColors getRandom() {
            kColors[] values = kColors.values();
            return values[ThreadLocalRandom.current().nextInt(values.length)];
        }
    }

    // General pattern config.
    public static final double kSpeedFactor = 1.0;

    // Moving bars pattern config.
    public static final int kBarSize = 5;

    // Gradient trail pattern config.
    public static final int kTrailCount = 1;
    public static final int kTrailSize = 150;

    // Rainbow pattern config.
    public static final int kRainbowFactor = 5;
}
