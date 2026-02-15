package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.kColors;
import frc.robot.subsystems.leds.patterns.*;
import java.util.List;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private RunnableLEDPattern currentPattern;
    private int currentPatternIndex = 0;
    private List<RunnableLEDPattern> patterns = List.of(
            new GradientFillPattern(this),
            new GradientTrailPattern(this),
            new MovingBarsPattern(this),
            new MovingRainbowFillPattern(this),
            new RainbowGradientTrailPattern(this));

    private int animationFrame = 0;
    private double realFrame = 0.0;

    private Color baseColor = new Color(0, 0, 100);
    private Color gradientColor = new Color(0, 255, 0);

    public LEDSubsystem() {
        led = new AddressableLED(kPWMChannel);
        buffer = new AddressableLEDBuffer(kLEDCount);

        led.setLength(kLEDCount);
        led.setData(buffer);
        led.start();

        currentPattern = patterns.get(currentPatternIndex);
    }

    @Override
    public void periodic() {
        clearLeds();

        if (currentPattern != null) {
            currentPattern.run();
        }

        updateLEDs();
        realFrame = (realFrame + kSpeedFactor) % kLEDCount;
        animationFrame = ((int) realFrame + kLEDCount) % kLEDCount;
    }

    public void setLEDColor(int ledIndex, Color color) {
        buffer.setRGB(ledIndex, (int) (color.red * 255.0), (int) (color.green * 255.0), (int) (color.blue * 255.0));
    }

    public void setLEDRGB(int ledIndex, int r, int g, int b) {
        buffer.setRGB(ledIndex, r, g, b);
    }

    public void setLEDHSV(int ledIndex, int h, int s, int v) {
        buffer.setHSV(ledIndex, h, s, v);
    }

    public void fillLEDs(Color color) {
        for (int i = 0; i < kLEDCount; i++) {
            setLEDColor(i, color);
        }
    }

    public void updateLEDs() {
        led.setData(buffer);
    }

    public void clearLeds() {
        fillLEDs(new Color(0, 0, 0));
    }

    public void setBaseColor(Color color) {
        this.baseColor = color;
    }

    public void setGradientColor(Color color) {
        this.gradientColor = color;
    }

    public Color getBaseColor() {
        return this.baseColor;
    }

    public Color getGradientColor() {
        return this.gradientColor;
    }

    public int getAnimationFrame() {
        return this.animationFrame;
    }

    /**
     * Linearly interpolates between two color doubles from 0.0-1.0 based on a ratio t and converts them to integers
     * from 0-255.
     *
     * @param start The starting color component value (0.0-1.0).
     * @param end The ending color component value (0.0-1.0).
     * @param t The interpolation ratio (0.0-1.0).
     * @return The interpolated integer value (0-255).
     */
    public int lerpColorComponent(double start, double end, double t) {
        return (int) ((start + (end - start) * t) * 255.0);
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
            setBaseColor(baseColor.color);
            setGradientColor(gradientColor.color);
        });
    }

    /**
     * Constructs a command to switch to the next pattern in the list of patterns.
     *
     * @return A command to switch to the next pattern in the list of patterns.
     */
    public Command nextPatternCommand() {
        return runOnce(() -> {
            currentPatternIndex = (currentPatternIndex + 1) % patterns.size();
            currentPattern = patterns.get(currentPatternIndex);
        });
    }
}
