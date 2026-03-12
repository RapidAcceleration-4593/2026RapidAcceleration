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
import frc.robot.subsystems.leds.LEDConstants.kColors;
import frc.robot.subsystems.leds.patterns.*;
import java.util.List;

public class LEDSubsystem extends SubsystemBase {

    private final BooleanTopic redAllianceTopic;
    private final BooleanSubscriber redAllianceSub;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private RunnableLEDPattern currentPattern;
    private RunnableLEDPattern currentOverlayPattern;
    private int currentPatternIndex = 0;
    // -1 = no overlay, 0-infinity = overlay.
    private int currentOverlayPatternIndex = -1;
    private List<RunnableLEDPattern> patterns = List.of(
            new GradientFillPattern(this),
            new GradientTrailPattern(this),
            new GradientCrossTrailPattern(this),
            new MovingRainbowFillPattern(this),
            new RainbowGradientTrailPattern(this));

    private int animationFrame = 0;
    private double realFrame = 0.0;

    private double speedFactor = 1.0;
    private double overlaySpeedFactor = 1.0;
    private Color baseColor = Color.kBlue;
    private Color overlayBaseColor = Color.kBlue;
    private Color gradientColor = Color.kBlack;
    private Color overlayGradientColor = Color.kBlack;
    private boolean useAllianceColor = true;
    private boolean overlayUseAllianceColor = true;

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

        if (currentOverlayPattern != null) {
            if (overlayUseAllianceColor) {
                overlayBaseColor = getAllianceColor();
                overlayGradientColor = Color.kBlack;
            }
            currentOverlayPattern.run();
        } else {
            if (useAllianceColor) {
                baseColor = getAllianceColor();
                gradientColor = Color.kBlack;
            }
            currentPattern.run();
        }

        updateLEDs();
        if (currentOverlayPattern == null) {
            realFrame = ((realFrame + kBaseSpeed * speedFactor) % kLEDCount + kLEDCount) % kLEDCount;
        } else {
            realFrame = ((realFrame + kBaseSpeed * overlaySpeedFactor) % kLEDCount + kLEDCount) % kLEDCount;
        }
        animationFrame = ((int) realFrame + kLEDCount) % kLEDCount;
    }

    public void setLEDColor(int ledIndex, Color color) {
        buffer.setLED(MathUtil.clamp(ledIndex, 0, kLEDCount - 1), color);
    }

    public void fillLEDs(Color color) {
        LEDPattern.solid(color).applyTo(buffer);
    }

    public void updateLEDs() {
        led.setData(buffer);
    }

    public void setPattern(int patternIndex) {
        currentPatternIndex = patternIndex % patterns.size();
        currentPattern = patterns.get(currentPatternIndex);
    }

    public void setOverlayPattern(int patternIndex) {
        if (patternIndex == -1) {
            currentOverlayPattern = null;
            return;
        }
        currentOverlayPatternIndex = patternIndex % patterns.size();
        currentOverlayPattern = patterns.get(currentOverlayPatternIndex);
    }

    public void setSpeed(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    public void setOverlaySpeed(double speedFactor) {
        this.overlaySpeedFactor = speedFactor;
    }

    public void setColor(Color baseColor, Color gradientColor) {
        this.baseColor = baseColor;
        this.gradientColor = gradientColor;
    }

    public void setOverlayColor(Color baseColor, Color gradientColor) {
        this.overlayBaseColor = baseColor;
        this.overlayGradientColor = gradientColor;
    }

    public void setUseAllianceColor(boolean value) {
        this.useAllianceColor = value;
    }

    public void setOverlayUseAllianceColor(boolean value) {
        this.overlayUseAllianceColor = value;
    }

    public Color getBaseColor() {
        if (currentOverlayPattern == null) {
            return this.baseColor;
        } else {
            return this.overlayBaseColor;
        }
    }

    public Color getGradientColor() {
        if (currentOverlayPattern == null) {
            return this.gradientColor;
        } else {
            return this.overlayGradientColor;
        }
    }

    public int getAnimationFrame() {
        return this.animationFrame;
    }

    public Color getAllianceColor() {
        boolean isRedAlliance = redAllianceSub.get(false);
        if (isRedAlliance) {
            return Color.kRed;
        } else {
            return Color.kBlue;
        }
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
