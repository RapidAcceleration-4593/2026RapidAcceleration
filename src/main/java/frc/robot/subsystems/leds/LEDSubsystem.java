package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    // General pattern config.
    private int patternIndex = 0;
    public int baseR = 0;
    public int baseG = 0;
    public int baseB = 100;
    public int gradientR = 0;
    public int gradientG = 255;
    public int gradientB = 0;

    public LEDSubsystem() {
		led = new AddressableLED(kPWMChannel);
		buffer = new AddressableLEDBuffer(kLEDCount);

        led.setLength(kLEDCount);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // movingBarsPattern();
        // gradientTrailPattern();
        // rainbowGradientTrailPattern();
        // movingRainbowFillPattern();
        gradientFillPattern();
    }

    public void setLedRgb(int ledIndex, int r, int g, int b) {
        buffer.setRGB(ledIndex, r, g, b);
    }

    public void setLedHsv(int ledIndex, int h, int s, int v) {
        buffer.setHSV(ledIndex, h, s, v);
    }

    /** Sets all LED's RGB values to a specified color.
	 * 
	 * @param r Red.
	 * @param g Green.
	 * @param b Blue.
	 */
    public void fillLeds(int r, int g, int b) {
        for(int i = 0; i < kLEDCount; i++) {
            setLedRgb(i, r, g, b);
        }
    }

    /** Updates all LEDs. */
    public void updateLeds() {
        led.setData(buffer);
    }

    /** Sets all LEDs' RGB values to zero. */
    public void clearLeds() {
		fillLeds(0, 0, 0);
    }

    /** Constructs a command to change the base and gradient colors for color specific patterns.
	 * 
	 * @param baseR The base red value.
	 * @param baseG The base green value.
	 * @param baseB The base blue value.
	 * @param gradientR The gradient red value.
	 * @param gradientG The gradient green value.
	 * @param gradientB The gradient blue value.
	 * @return A commmand to change the base and gradient colors.
	 */
    public Command changeColorCommand(int baseR, int baseG, int baseB, int gradientR, int gradientG, int gradientB) {
        return runOnce(() -> {
            this.baseR = baseR;
            this.baseG = baseG;
            this.baseB = baseB;
            this.gradientR = gradientR;
            this.gradientG = gradientG;
            this.gradientB = gradientB;
        });
    }

    /** Fills every LED with a moving rainbow pattern. */
    public void movingRainbowFillPattern() {
        clearLeds();
        for(int i = 0; i < kLEDCount; i++) {
            double progress = (double)i / kLEDCount;
            int hue = (int)((progress + (double)patternIndex / kLEDCount) * 180.0 * kRainbowFactor) % 180;
            setLedHsv(i, hue, 255, 255);
        }
        updateLeds();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }

	/** Fills the LEDs with a continuous gradient pattern. */
    public void gradientFillPattern() {
        clearLeds();
        for(int i = 0; i < kTrailSize; i++) {
            int pos = (patternIndex - i + kLEDCount) % kLEDCount;
            int r = (int)(baseR * Math.abs(1.0 - (double)i / kLEDCount * 2.0));
            int b = (int)(baseB * Math.abs(1.0 - (double)i / kLEDCount * 2.0));
            int g = (int)(baseG * Math.abs(1.0 - (double)i / kLEDCount * 2.0));
            r += (int)(gradientR * (Math.abs(1.0 - (double)i / kLEDCount * 2.0) * -1.0 + 1.0));
            g += (int)(gradientG * (Math.abs(1.0 - (double)i / kLEDCount * 2.0) * -1.0 + 1.0));
            b += (int)(gradientB * (Math.abs(1.0 - (double)i / kLEDCount * 2.0) * -1.0 + 1.0));
            setLedRgb(pos, r, g, b);
        }
        updateLeds();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }

    /** Makes individual rainbow trails. */
    public void rainbowGradientTrailPattern() {
        clearLeds();
        for(int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for(int i = 0; i < kTrailSize; i++) {
                int pos = (patternIndex + current_trail - i + kLEDCount) % kLEDCount;
                double progress = (float)pos / kLEDCount;
                int hue = (int)(progress * 180.0 * kRainbowFactor) % 180;
                int brightness = (int)(255 * (1.0 - (double)i / (double)kTrailSize));
                setLedHsv(pos, hue, 255, brightness);
            }
        }
        updateLeds();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }

    /** Makes individual gradient trails. */
    public void gradientTrailPattern() {
        clearLeds();
        for(int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for(int i = 0; i < kTrailSize; i++) {
                int pos = (patternIndex + current_trail - i + kLEDCount) % kLEDCount;
                int r = (int)(baseR * (1.0 - (double)i / (double)kTrailSize));
                int g = (int)(baseG * (1.0 - (double)i / (double)kTrailSize));
                int b = (int)(baseB * (1.0 - (double)i / (double)kTrailSize));
                r += (int)(gradientR * ((double)i / (double)kTrailSize));
                g += (int)(gradientG * ((double)i / (double)kTrailSize));
                b += (int)(gradientB * ((double)i / (double)kTrailSize));
                setLedRgb(pos, r, g, b);
            }
        }
        updateLeds();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }

    /** Makes moving solid bars equally spaced apart of one specific color. */
    public void movingBarsPattern() {
        clearLeds();
        for(int i = 0; i < kLEDCount; i++) {
            if(((i + patternIndex) / kBarSize) % 2 == 0) {
                setLedRgb(i, baseR, baseG, baseB);
            }
        }
        updateLeds();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }
}
