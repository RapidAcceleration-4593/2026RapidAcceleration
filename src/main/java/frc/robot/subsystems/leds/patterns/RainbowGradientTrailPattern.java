package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import frc.robot.subsystems.leds.LEDSubsystem;

public class RainbowGradientTrailPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public RainbowGradientTrailPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int current_trail = 0; current_trail < kLEDCount; current_trail += kLEDCount / kTrailCount) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (subsystem.getPatternIndex() + current_trail - i + kLEDCount) % kLEDCount;

                double progress = (double) pos / kLEDCount;

                int hue = (int) (progress * 180.0 * kRainbowFactor) % 180;
                int brightness = (int) (255 * (1.0 - (double) i / (double) kTrailSize));

                subsystem.setLEDHSV(pos, hue, 255, brightness);
            }
        }
    }
}
