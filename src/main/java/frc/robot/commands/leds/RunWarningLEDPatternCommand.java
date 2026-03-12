package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunWarningLEDPatternCommand extends Command {

    private final LEDSubsystem subsystem;

    public RunWarningLEDPatternCommand(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setOverlayPattern(2);
        subsystem.setOverlaySpeed(3.0);
        subsystem.setOverlayUseAllianceColor(false);
        subsystem.setOverlayColor(Color.kGreen, Color.kBlack);
    }

    @Override
    public void end(boolean interupted) {
        subsystem.setOverlayPattern(-1);
        subsystem.setOverlaySpeed(1.0);
        subsystem.setOverlayUseAllianceColor(true);
    }
}
