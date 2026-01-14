package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RotateTurretCommand extends Command {
	
	private final TurretSubsystem turret;
	private final double speed;

	public RotateTurretCommand(TurretSubsystem turretSubsystem, double speed) {
		this.turret = turretSubsystem;
		this.speed = speed;
		addRequirements(turretSubsystem);
	}

	@Override
	public void execute() {
		turret.setMotorSpeed(speed);
	}

	@Override
	public void end(boolean interrupted) {
		turret.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
