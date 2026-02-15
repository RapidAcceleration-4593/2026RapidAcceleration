package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import frc.robot.subsystems.leds.LEDSubsystem;

public class GradientFillPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public GradientFillPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int i = 0; i < kTrailSize; i++) {
            int pos = (subsystem.getPatternIndex() - i + kLEDCount) % kLEDCount;

            double fadeRatio = Math.abs(1.0 - (2.0 * i / kLEDCount));

            int r = subsystem.lerpColorComponent(
                    subsystem.getBaseColor().red, subsystem.getGradientColor().red, fadeRatio);
            int g = subsystem.lerpColorComponent(
                    subsystem.getBaseColor().green, subsystem.getGradientColor().green, fadeRatio);
            int b = subsystem.lerpColorComponent(
                    subsystem.getBaseColor().blue, subsystem.getGradientColor().blue, fadeRatio);

            subsystem.setLEDRGB(pos, r, g, b);
        }
    }
}
