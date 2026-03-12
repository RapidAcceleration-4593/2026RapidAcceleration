package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunShooterLEDPatternCommand extends Command {

    private final LEDSubsystem subsystem;

    public RunShooterLEDPatternCommand(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPattern(2);
        subsystem.setSpeed(3.0);
        subsystem.setUseAllianceColor(false);
        subsystem.setColor(Color.kGreen, Color.kBlack);
    }

    @Override
    public void end(boolean interupted) {
        subsystem.setPattern(0);
        subsystem.setSpeed(1.0);
        subsystem.setUseAllianceColor(true);
    }
}
