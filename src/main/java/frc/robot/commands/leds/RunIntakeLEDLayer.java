package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDLayer;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunIntakeLEDLayer extends Command {

    private final LEDSubsystem subsystem;

    private final LEDLayer layer;

    public RunIntakeLEDLayer(LEDSubsystem subsystem) {
        this.subsystem = subsystem;

        layer = new LEDLayer(1.0, 2, Color.kOrangeRed, Color.kBlack, false, -1.0);
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
