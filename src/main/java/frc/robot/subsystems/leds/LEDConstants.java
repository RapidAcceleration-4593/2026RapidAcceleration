package frc.robot.subsystems.leds;

import java.util.concurrent.ThreadLocalRandom;

public final class LEDConstants {

    public static final int kPWMChannel = 0;
    public static final int kLEDCount = 150;

    public static enum kColors {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 70, 0),
        ORANGE(255, 30, 0),
        PURPLE(255, 0, 255);
        public final int r, g, b;

        kColors(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
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
