package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDSubsystem;

public class GradientCrossTrailPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public GradientCrossTrailPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        int HalfLEDCount = kLEDCount / 2;

        for (int current_trail = 0; current_trail < kTrailCount; current_trail += 1) {
            for (int i = 0; i < kTrailSize; i++) {
                int pos = (subsystem.getAnimationFrame()
                                + (current_trail * HalfLEDCount / kTrailCount)
                                - i
                                + HalfLEDCount)
                        % HalfLEDCount;

                double fadeRatio = Math.min(1.0, (double) i / kTrailSize);
                fadeRatio = Math.pow(fadeRatio, 3.0);

                Color color = Color.lerpRGB(subsystem.getBaseColor(), subsystem.getGradientColor(), fadeRatio);

                subsystem.setLEDColor(-pos + HalfLEDCount, color);
                subsystem.setLEDColor(pos + HalfLEDCount, color);
            }
        }
    }
}
