package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deploy.DeployConstants;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem intake;
    private final DeploySubsystem deploy;

    public IntakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        this.intake = intake;
        this.deploy = deploy;
        addRequirements(intake, deploy);
    }

    @Override
    public void initialize() {
        deploy.goToDistanceCommand(DeployConstants.kExtendedDistance);
    }

    @Override
    public void execute() {
        if (deploy.atTargetDistance()) {
            intake.runCommand();
        } else {
            intake.stopCommand();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopCommand();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
