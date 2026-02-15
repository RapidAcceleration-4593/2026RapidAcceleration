package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import frc.robot.subsystems.leds.LEDSubsystem;

public class GradientTrailPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public GradientTrailPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (subsystem.getAnimationFrame() + current_trail - i + kLEDCount) % kLEDCount;

                double fadeRatio = (double) i / kTrailSize;

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
}
