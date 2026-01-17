package frc.robot.commands.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends Command {

    private final IntakeSubsystem intake;

    public RunIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(kIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
