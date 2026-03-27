package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDLayer;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunIntakeLEDPatternCommand extends Command {

    private final LEDSubsystem subsystem;

    private LEDLayer layer;

    public RunIntakeLEDPatternCommand(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        layer = subsystem.addLayer(1.0);
        layer.useAllianceColor = false;
        layer.baseColor = Color.kOrangeRed;
        layer.gradientColor = Color.kBlack;
        layer.patternIndex = 2;
        layer.speedFactor = -1.0;
    }

    @Override
    public void end(boolean interupted) {
        subsystem.removeLayer(layer);
    }
}
