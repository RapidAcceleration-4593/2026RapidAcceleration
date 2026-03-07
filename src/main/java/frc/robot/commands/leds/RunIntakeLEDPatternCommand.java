package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunIntakeLEDPatternCommand extends Command {

    private final LEDSubsystem subsystem;

    public RunIntakeLEDPatternCommand(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.changePattern(2);
        subsystem.changeSpeed(-1.0);
        subsystem.setUseAllianceColor(false);
        subsystem.changeColor(Color.kOrangeRed, Color.kBlack);
    }

    @Override
    public void end(boolean interupted) {
        subsystem.changePattern(0);
        subsystem.changeSpeed(1.0);
        subsystem.setUseAllianceColor(true);
    }
}
