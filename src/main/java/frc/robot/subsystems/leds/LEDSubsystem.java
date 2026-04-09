package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.patterns.*;
import java.util.ArrayList;
import java.util.List;

public class LEDSubsystem extends SubsystemBase {

    private final BooleanTopic redAllianceTopic;
    private final BooleanSubscriber redAllianceSub;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private RunnableLEDPattern currentPattern;
    private List<RunnableLEDPattern> patterns = List.of(
            new GradientFillPattern(this),
            new GradientTrailPattern(this),
            new GradientCrossTrailPattern(this),
            new MovingRainbowFillPattern(this),
            new RainbowGradientTrailPattern(this),
            new BlinkPattern(this));

    private int animationFrame = 0;
    private double realFrame = 0.0;

    private double speedFactor = 1.0;
    private Color baseColor = Color.kBlue;
    private Color gradientColor = Color.kBlack;
    private boolean useAllianceColor = true;
    private int currentPatternIndex = 0;

    private List<LEDLayer> layers = new ArrayList<>();
    private final LEDLayer defaultLayer = new LEDLayer(-1.0, 0, Color.kBlack, Color.kBlack, true, 1.0);

    public LEDSubsystem() {
        redAllianceTopic = NetworkTableInstance.getDefault().getBooleanTopic("/FMSInfo/IsRedAlliance");
        redAllianceSub = redAllianceTopic.subscribe(false);

        led = new AddressableLED(kPWMChannel);
        buffer = new AddressableLEDBuffer(kLEDCount);

        led.setLength(kLEDCount);
        led.setData(buffer);
        led.start();

        addLayer(defaultLayer);
        updateLayers();
    }

    @Override
    public void periodic() {
        fillLEDs(Color.kBlack);

        if (useAllianceColor) {
            baseColor = getAllianceColor();
            gradientColor = Color.kBlack;
        }
        currentPattern.run();

        updateLEDs();
        realFrame = ((realFrame + kBaseSpeed * speedFactor) % kLEDCount + kLEDCount) % kLEDCount;
        animationFrame = ((int) realFrame + kLEDCount) % kLEDCount;
    }

    public void setLEDColor(int ledIndex, Color color) {
        buffer.setLED(MathUtil.clamp(ledIndex, 0, kLEDCount - 1), color);
    }

    public void fillLEDs(Color color) {
        for (int i = 0; i < kLEDCount; i++) {
            buffer.setLED(i, color);
        }
    }

    public void updateLEDs() {
        led.setData(buffer);
    }

    public void setPattern(int patternIndex) {
        currentPatternIndex = patternIndex % patterns.size();
        currentPattern = patterns.get(currentPatternIndex);
    }

    public void setSpeed(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    public void setColor(Color baseColor, Color gradientColor) {
        this.baseColor = baseColor;
        this.gradientColor = gradientColor;
    }

    public void setUseAllianceColor(boolean value) {
        this.useAllianceColor = value;
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

    public Color getAllianceColor() {
        boolean isRedAlliance = redAllianceSub.get(false);
        if (isRedAlliance) {
            return Color.kRed;
        } else {
            return Color.kBlue;
        }
    }

    public void addLayer(LEDLayer layer) {
        layers.add(layer);
        updateLayers();
    }

    public void removeLayer(LEDLayer layer) {
        layers.remove(layer);
        updateLayers();
    }

    /**
     * Sorts layer list based on priority and sets all LED subsystem properties to highest priority layer properties.
     */
    private void updateLayers() {
        layers.sort((a, b) -> Double.compare(b.getPriority(), a.getPriority()));

        LEDLayer layer = layers.get(0);
        setPattern(layer.getPatternIndex());
        setColor(layer.getBaseColor(), layer.getGradientcColor());
        setUseAllianceColor(layer.getUseAllianceColor());
        setSpeed(layer.getSpeedFactor());
    }
}
