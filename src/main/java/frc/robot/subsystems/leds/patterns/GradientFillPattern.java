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
        // subsystem.applyPattern(
        //         LEDPattern.gradient(GradientType.kContinuous, subsystem.getBaseColor(), subsystem.getGradientColor())
        //                 .offsetBy(subsystem.getAnimationFrame()));

        for (int i = 0; i < kLEDCount; i++) {
            int pos = (subsystem.getAnimationFrame() - i + kLEDCount) % kLEDCount;

            double x = (double) i / kLEDCount;
            double fadeRatio = (1 - Math.cos(2 * Math.PI * x)) / 2.0;

            Color color = Color.lerpRGB(subsystem.getBaseColor(), subsystem.getGradientColor(), fadeRatio);

            subsystem.setLEDColor(pos, color);
        }
    }
}
