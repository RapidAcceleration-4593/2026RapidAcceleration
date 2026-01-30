package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deploy.DeployConstants;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class ClimbCommand extends Command {

    private final ClimberSubsystem climber;
    private final DeploySubsystem deploy;

    public ClimbCommand(ClimberSubsystem climber, DeploySubsystem deploy) {
        this.climber = climber;
        this.deploy = deploy;
        addRequirements(climber, deploy);

        Trigger topTrigger = new Trigger(
                () -> (climber.atTargetDistance() && climber.getTargetDistance() == ClimberConstants.kMaximumDistance));
        topTrigger.onTrue(climber.goToDistanceCommand(ClimberConstants.kMinimumDistance));
    }

    @Override
    public void initialize() {
        climber.goToDistanceCommand(ClimberConstants.kMaximumDistance);
        deploy.goToDistanceCommand(DeployConstants.kRetractedDistance);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
