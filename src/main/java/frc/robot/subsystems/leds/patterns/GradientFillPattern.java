package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDSubsystem;

public class GradientFillPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public GradientFillPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int i = 0; i < kTrailSize; i++) {
            int pos = (subsystem.getAnimationFrame() - i + kLEDCount) % kLEDCount;

            double fadeRatio = Math.abs(1.0 - (2.0 * i / kLEDCount));

            Color color = Color.lerpRGB(subsystem.getBaseColor(), subsystem.getGradientColor(), fadeRatio);

            subsystem.setLEDColor(pos, color);
        }
    }
}
