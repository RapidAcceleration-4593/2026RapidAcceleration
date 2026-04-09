package frc.robot.subsystems.leds.patterns;

import frc.robot.subsystems.leds.LEDSubsystem;

public class BlinkPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public BlinkPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        int animationFrame = subsystem.getAnimationFrame();
        if (Math.floor(animationFrame / 10.0) % 2 == 0) {
            subsystem.fillLEDs(subsystem.getBaseColor());
        }
    }
}
