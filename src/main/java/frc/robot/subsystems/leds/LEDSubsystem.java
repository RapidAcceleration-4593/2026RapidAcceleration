package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.kColors;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private int patternIndex = 0;
	private double realIndex = 0.0;
    private int baseR = 0;
    private int baseG = 0;
    private int baseB = 100;
    private int gradientR = 0;
    private int gradientG = 255;
    private int gradientB = 0;

    public LEDSubsystem() {
        led = new AddressableLED(kPWMChannel);
        buffer = new AddressableLEDBuffer(kLEDCount);

        led.setLength(kLEDCount);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        clearLeds();

        // movingBarsPattern();
        // gradientTrailPattern();
        // rainbowGradientTrailPattern();
        // movingRainbowFillPattern();
        gradientFillPattern();

        updateLEDs();
		realIndex = (realIndex + kSpeedFactor) % kLEDCount;
        patternIndex = ((int) realIndex + kLEDCount) % kLEDCount;
    }

    public void setLEDRGB(int ledIndex, int r, int g, int b) {
        buffer.setRGB(ledIndex, r, g, b);
    }

    public void setLEDHSV(int ledIndex, int h, int s, int v) {
        buffer.setHSV(ledIndex, h, s, v);
    }

    public void fillLEDs(int r, int g, int b) {
        for (int i = 0; i < kLEDCount; i++) {
            setLEDRGB(i, r, g, b);
        }
    }

    public void updateLEDs() {
        led.setData(buffer);
    }

    public void clearLeds() {
        fillLEDs(0, 0, 0);
    }

    public void setBaseRGB(int r, int g, int b) {
        this.baseR = r;
        this.baseG = g;
        this.baseB = b;
    }

    public void setGradientRGB(int r, int g, int b) {
        this.gradientR = r;
        this.gradientG = g;
        this.gradientB = b;
    }

	private int lerp(int a, int b, double t) {
		return (int) (a + (b - a) * t);
	}

    /**
     * Constructs a command to change the base and gradient colors for color specific patterns.
     *
     * @param baseColor The base color.
     * @param gradientColor The gradient color.
     * @return A commmand to change the base and gradient colors.
     */
    public Command changeColorCommand(kColors baseColor, kColors gradientColor) {
        return runOnce(() -> {
            setBaseRGB(baseColor.r, baseColor.g, baseColor.b);
            setGradientRGB(gradientColor.r, gradientColor.g, gradientColor.b);
        });
    }

    /** Fills every LED with a moving rainbow pattern. */
    public void movingRainbowFillPattern() {
        for (int i = 0; i < kLEDCount; i++) {
            double progress = (double) i / kLEDCount;

            int hue = (int) ((progress + (double) patternIndex / kLEDCount) * 180.0 * kRainbowFactor) % 180;

            setLEDHSV(i, hue, 255, 255);
        }
    }

    /** Fills the LEDs with a continuous gradient pattern. */
    public void gradientFillPattern() {
        for (int i = 0; i < kTrailSize; i++) {
            int pos = (patternIndex - i + kLEDCount) % kLEDCount;

            double fadeRatio = Math.abs(1.0 - (2.0 * i / kLEDCount));

            int r = lerp(baseR, gradientR, fadeRatio);
            int g = lerp(baseG, gradientG, fadeRatio);
            int b = lerp(baseB, gradientB, fadeRatio);

            setLEDRGB(pos, r, g, b);
        }
    }

    /** Makes individual rainbow trails. */
    public void rainbowGradientTrailPattern() {
        for (int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (patternIndex + current_trail - i + kLEDCount) % kLEDCount;

                double progress = (double) pos / kLEDCount;

                int hue = (int) (progress * 180.0 * kRainbowFactor) % 180;
                int brightness = (int) (255 * (1.0 - (double) i / (double) kTrailSize));

                setLEDHSV(pos, hue, 255, brightness);
            }
        }
    }

    /** Makes individual gradient trails. */
    public void gradientTrailPattern() {
        for (int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (patternIndex + current_trail - i + kLEDCount) % kLEDCount;

                double fadeRatio = (double) i / kTrailSize;

                int r = lerp(baseR, gradientR, fadeRatio);
                int g = lerp(baseG, gradientG, fadeRatio);
                int b = lerp(baseB, gradientB, fadeRatio);

                setLEDRGB(pos, r, g, b);
            }
        }
    }

    /** Makes moving solid bars equally spaced apart of one specific color. */
    public void movingBarsPattern() {
        for (int i = 0; i < kLEDCount; i++) {
            if (((i + patternIndex) / kBarSize) % 2 == 0) {
                setLEDRGB(i, baseR, baseG, baseB);
            }
        }
    }
}
