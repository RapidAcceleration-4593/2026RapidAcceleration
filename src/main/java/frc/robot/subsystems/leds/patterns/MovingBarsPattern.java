package frc.robot.subsystems.leds.patterns;

import static frc.robot.subsystems.leds.LEDConstants.*;

import frc.robot.subsystems.leds.LEDSubsystem;

public class MovingBarsPattern implements RunnableLEDPattern {

    private final LEDSubsystem subsystem;

    public MovingBarsPattern(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void run() {
        for (int i = 0; i < kLEDCount; i++) {
            if (((i + subsystem.getAnimationFrame()) / kBarSize) % 2 == 0) {
                subsystem.setLEDColor(i, subsystem.getBaseColor());
            }
        }
    }
}
