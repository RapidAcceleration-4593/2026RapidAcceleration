package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDSubsystem;

public class GradientTrailPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public GradientTrailPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int current_trail = 0; current_trail < kTrailCount; current_trail += 1) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (subsystem.getAnimationFrame() + (current_trail * kLEDCount / kTrailCount) - i + kLEDCount)
                        % kLEDCount;

                double fadeRatio = Math.min(1.0, (double) i / kTrailSize);
                fadeRatio = Math.pow(fadeRatio, 3.0);

                Color color = Color.lerpRGB(subsystem.getBaseColor(), subsystem.getGradientColor(), fadeRatio);

                subsystem.setLEDColor(pos, color);
            }
        }
    }
}
