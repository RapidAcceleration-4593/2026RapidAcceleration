package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideCenterTrench extends AutonCommand {

    public SideCenterTrench(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterTrench-1", "SideCenterTrench-2"));
        EventTrigger shootTrigger = new EventTrigger("StartShooter");

        addCommands(
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(0)),
                        Commands.sequence(
                                NamedCommands.getCommand("ExtendDeployCommand"),
                                NamedCommands.getCommand("RetractDeployCommand"),
                                NamedCommands.getCommand("IntakeCommand").until(shootTrigger),
                                Commands.deadline(
                                        Commands.waitSeconds(10.0),
                                        NamedCommands.getCommand("ShootCommand"),
                                        Commands.sequence(
                                                Commands.waitSeconds(4.0),
                                                NamedCommands.getCommand("ShakeDeployCommand"))))),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))));
    }
}
