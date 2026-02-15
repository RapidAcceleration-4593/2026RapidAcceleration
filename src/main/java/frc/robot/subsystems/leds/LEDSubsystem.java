package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

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

    public void setLEDRGB(int ledIndex, int r, int g, int b) {
        buffer.setRGB(ledIndex, r, g, b);
    }

    public void setLEDHSV(int ledIndex, int h, int s, int v) {
        buffer.setHSV(ledIndex, h, s, v);
    }

    /** Sets all LED's RGB values to a specified color.
	 * 
	 * @param r Red.
	 * @param g Green.
	 * @param b Blue.
	 */
    public void fillLEDs(int r, int g, int b) {
        for(int i = 0; i < kLEDCount; i++) {
            setLEDRGB(i, r, g, b);
        }
    }

    /** Updates all LEDs. */
    public void updateLEDs() {
        led.setData(buffer);
    }

    /** Sets all LEDs' RGB values to zero. */
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

    /** Constructs a command to change the base and gradient colors for color specific patterns.
	 * 
	 * @param baseColor The base color.
	 * @param gradientColor The gradient color.
	 * @return A commmand to change the base and gradient colors.
	 */
    public Command changeColorCommand(kColors baseColor, kColors gradientColor) {
        return runOnce(() -> {
			switch (baseColor) {
				case RED -> setBaseRGB(255, 0, 0);
				case GREEN -> setBaseRGB(0, 255, 0);
				case BLUE -> setBaseRGB(0, 0, 255);
				case YELLOW -> setBaseRGB(255, 70, 0);
				case ORANGE -> setBaseRGB(255, 30, 0);
			}
			switch (gradientColor) {
				case RED -> setGradientRGB(255, 0, 0);
				case GREEN -> setGradientRGB(0, 255, 0);
				case BLUE -> setGradientRGB(0, 0, 255);
				case YELLOW -> setGradientRGB(255, 70, 0);
				case ORANGE -> setGradientRGB(255, 30, 0);
			}
        });
    }

    /** Fills every LED with a moving rainbow pattern. */
    public void movingRainbowFillPattern() {
        clearLeds();
        for(int i = 0; i < kLEDCount; i++) {
            double progress = (double)i / kLEDCount;
            int hue = (int)((progress + (double)patternIndex / kLEDCount) * 180.0 * kRainbowFactor) % 180;
            setLEDHSV(i, hue, 255, 255);
        }
        updateLEDs();
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
            setLEDRGB(pos, r, g, b);
        }
        updateLEDs();
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
                setLEDHSV(pos, hue, 255, brightness);
            }
        }
        updateLEDs();
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
                setLEDRGB(pos, r, g, b);
            }
        }
        updateLEDs();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }

    /** Makes moving solid bars equally spaced apart of one specific color. */
    public void movingBarsPattern() {
        clearLeds();
        for(int i = 0; i < kLEDCount; i++) {
            if(((i + patternIndex) / kBarSize) % 2 == 0) {
                setLEDRGB(i, baseR, baseG, baseB);
            }
        }
        updateLEDs();
        patternIndex = (patternIndex + 1) % kLEDCount;
    }
}
