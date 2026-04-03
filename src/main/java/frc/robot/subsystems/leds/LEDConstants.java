package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import java.util.Random;

public final class LEDConstants {

    public static final int kPWMChannel = 0;
    public static final int kLEDCount = 88;

    public static enum kColors {
        RED(Color.kRed),
        GREEN(Color.kGreen),
        BLUE(Color.kBlue),
        YELLOW(Color.kYellow),
        ORANGE(Color.kOrange),
        PURPLE(Color.kPurple),
        BLACK(Color.kBlack),
        WHITE(Color.kWhite);
        public final Color color;

        kColors(Color color) {
            this.color = color;
        }

        private static final Random random = new Random();

        public static Color getRandom() {
            kColors[] values = kColors.values();
            return values[random.nextInt(values.length)].color;
        }
    }

    // General pattern config.
    public static final double kBaseSpeed = 1.0;

    // Trail pattern config.
    public static final int kTrailCount = 1;
    public static final int kTrailSize = 30;

    // Rainbow pattern config.
    public static final double kRainbowFactor = 2.0;
}
