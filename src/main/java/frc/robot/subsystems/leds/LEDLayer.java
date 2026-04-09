package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDLayer {

    private final double priority;

    private final int patternIndex;
    private final Color baseColor;
    private final Color gradientColor;
    private final boolean useAllianceColor;
    private final double speedFactor;

    public LEDLayer(
            double priority,
            int patternIndex,
            Color baseColor,
            Color gradientcColor,
            boolean useAllianceColor,
            double speedFactor) {
        this.priority = priority;

        this.patternIndex = patternIndex;
        this.baseColor = baseColor;
        this.gradientColor = gradientcColor;
        this.useAllianceColor = useAllianceColor;
        this.speedFactor = speedFactor;
    }

    public int getPatternIndex() {
        return patternIndex;
    }

    public Color getBaseColor() {
        return baseColor;
    }

    public Color getGradientcColor() {
        return gradientColor;
    }

    public boolean getUseAllianceColor() {
        return useAllianceColor;
    }

    public double getSpeedFactor() {
        return speedFactor;
    }

    public double getPriority() {
        return priority;
    }
}
