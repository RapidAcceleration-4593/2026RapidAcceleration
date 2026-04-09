package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDLayer;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunWarningLEDLayer extends Command {

    private final LEDSubsystem subsystem;

    private final LEDLayer layer;

    public RunWarningLEDLayer(LEDSubsystem subsystem) {
        this.subsystem = subsystem;

        layer = new LEDLayer(2.0, 5, Color.kRed, Color.kRed, false, 2.0);
    }

    @Override
    public void initialize() {
        subsystem.addLayer(layer);
    }

    @Override
    public void end(boolean interupted) {
        subsystem.removeLayer(layer);
    }
}
