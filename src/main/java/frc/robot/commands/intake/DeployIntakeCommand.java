package frc.robot.commands.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DeployIntakeCommand extends Command {

    private final IntakeSubsystem intake;

    public DeployIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setDeploySpeed(kDeploySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopDeploy();
    }

    @Override
    public boolean isFinished() {
        // return intake.isDeployed();
        return false;
    }
}
