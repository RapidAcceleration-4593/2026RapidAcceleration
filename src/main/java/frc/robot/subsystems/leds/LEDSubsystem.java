package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.patterns.*;
import java.util.List;

public class LEDSubsystem extends SubsystemBase {

    private final BooleanTopic redAllianceTopic;
    private final BooleanSubscriber redAllianceSub;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private RunnableLEDPattern currentPattern;
    private int currentPatternIndex = 0;
    private List<RunnableLEDPattern> patterns = List.of(
            new GradientFillPattern(this),
            new GradientTrailPattern(this),
            new GradientCrossTrailPattern(this),
            new MovingRainbowFillPattern(this),
            new RainbowGradientTrailPattern(this));

    private int animationFrame = 0;
    private double realFrame = 0.0;
    private double speedFactor = 1.0;

    private Color baseColor = Color.kBlue;
    private Color gradientColor = Color.kGreen;

    public LEDSubsystem() {
        redAllianceTopic = NetworkTableInstance.getDefault().getBooleanTopic("/FMSInfo/IsRedAlliance");
        redAllianceSub = redAllianceTopic.subscribe(false);

        led = new AddressableLED(kPWMChannel);
        buffer = new AddressableLEDBuffer(kLEDCount);

        led.setLength(kLEDCount);
        led.setData(buffer);
        led.start();

        currentPattern = patterns.get(currentPatternIndex);
    }

    @Override
    public void periodic() {
        fillLEDs(Color.kBlack);
        boolean isRedAlliance = redAllianceSub.get(false);

        if (isRedAlliance) {
            baseColor = Color.kRed;
            gradientColor = Color.kBlack;
        } else {
            baseColor = Color.kBlue;
            gradientColor = Color.kBlack;
        }

        if (currentPattern != null) {
            currentPattern.run();
        }

        updateLEDs();
        realFrame = (realFrame + kBaseSpeed * speedFactor) % kLEDCount;
        animationFrame = ((int) realFrame + kLEDCount) % kLEDCount;
    }

    public void setLEDColor(int ledIndex, Color color) {
        buffer.setLED(MathUtil.clamp(ledIndex, 0, kLEDCount - 1), color);
    }

    public void fillLEDs(Color color) {
        LEDPattern.solid(color).applyTo(buffer);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    public void updateLEDs() {
        led.setData(buffer);
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

	public void changePattern(int patternIndex) {
		currentPatternIndex = patternIndex % patterns.size();
        currentPattern = patterns.get(currentPatternIndex);
	}

	public void changeSpeed(double speedFactor) {
		this.speedFactor = speedFactor;
	}

    /**
     * Constructs a command to stop all LED patterns and turn off all LEDs.
     *
     * @return A command to stop all LED patterns and turn off all LEDs.
     */
    public Command stopLEDs() {
        return runOnce(() -> {
            fillLEDs(Color.kBlack);
        });
    }

    /**
     * Constructs a command to change the base and gradient colors for color specific patterns.
     *
     * @param baseColor The base color.
     * @param gradientColor The gradient color.
     * @return A commmand to change the base and gradient colors.
     */
    public Command changeColorCommand(Color baseColor, Color gradientColor) {
        return runOnce(() -> {
            this.baseColor = baseColor;
            this.gradientColor = gradientColor;
        });
    }

    /**
     * Constructs a command to change the base and gradient colors to random colors.
     *
     * @return A commmand to change the base and gradient colors.
     */
    public Command randomColorCommand() {
        return runOnce(() -> {
            this.baseColor = kColors.getRandom();
            this.gradientColor = kColors.getRandom();
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
