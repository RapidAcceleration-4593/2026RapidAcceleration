package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDSubsystem;

public class MovingRainbowFillPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public MovingRainbowFillPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int i = 0; i < kLEDCount; i++) {
            double progress = (double) i / kLEDCount;

            int hue = (int) ((progress + (double) subsystem.getAnimationFrame() / kLEDCount) * 180.0 * kRainbowFactor)
                    % 180;

            subsystem.setLEDColor(i, Color.fromHSV(hue, 255, 255));
        }
    }
}
