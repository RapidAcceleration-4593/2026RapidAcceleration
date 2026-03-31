package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDLayer {

    private final int patternIndex;
    private final Color baseColor;
    private final Color gradientColor;
    private final boolean useAllianceColor;
    private final double speedFactor;

    private final double priority;

    public LEDLayer(
            double priority,
            int patternIndex,
            Color baseColor,
            Color gradientcColor,
            boolean useAllianceColor,
            double speedFactor) {
        this.patternIndex = patternIndex;
        this.baseColor = baseColor;
        this.gradientColor = gradientcColor;
        this.useAllianceColor = useAllianceColor;
        this.speedFactor = speedFactor;

        this.priority = priority;
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
