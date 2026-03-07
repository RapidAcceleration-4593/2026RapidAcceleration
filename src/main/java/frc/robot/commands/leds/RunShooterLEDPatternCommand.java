package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RunShooterLEDPatternCommand extends Command{
	
	private final LEDSubsystem subsystem;

	public RunShooterLEDPatternCommand(LEDSubsystem subsystem) {
		this.subsystem = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		subsystem.changePattern(2);
		subsystem.changeSpeed(2.0);
		subsystem.changeColor(Color.kGreen, Color.kBlack);
	}

	@Override
	public void end(boolean interupted) {
		subsystem.changePattern(0);
		subsystem.changeSpeed(1.0);
		subsystem.changeColor(Color.kBlue, Color.kBlack);
	}
}
