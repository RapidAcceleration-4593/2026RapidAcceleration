package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Side2xCenterTrench extends AutonCommand {

    public Side2xCenterTrench(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterTrench-1", "SideCenterTrench-2", "SideCenterTrench-3"));
        EventTrigger shootTrigger = new EventTrigger("StartShooter");

        addCommands(
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(0)),
                        Commands.sequence(
                                NamedCommands.getCommand("ExtendDeployCommand"),
                                NamedCommands.getCommand("RetractDeployCommand"),
                                NamedCommands.getCommand("IntakeCommand").until(shootTrigger),
                                Commands.deadline(
                                        Commands.waitSeconds(5.0),
                                        NamedCommands.getCommand("ShootCommand"),
                                        Commands.sequence(
                                                Commands.waitSeconds(2.0),
                                                NamedCommands.getCommand("ShakeDeployCommand"))))),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))),
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(2)),
                        Commands.waitUntil(shootTrigger)
                                .andThen(Commands.deadline(
                                        Commands.waitSeconds(5.0),
                                        NamedCommands.getCommand("ShootCommand"),
                                        Commands.sequence(
                                                Commands.waitSeconds(2.0),
                                                NamedCommands.getCommand("ShakeDeployCommand"))))));
    }
}
