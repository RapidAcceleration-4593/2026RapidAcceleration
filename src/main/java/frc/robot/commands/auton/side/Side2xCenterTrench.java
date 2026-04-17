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
                        Commands.deadline(
                                Commands.waitUntil(shootTrigger),
                                NamedCommands.getCommand("ExtendDeployCommand")
                                        .andThen(NamedCommands.getCommand("RetractDeployCommand"))
                                        .andThen(NamedCommands.getCommand("IntakeCommand"))),
                        Commands.waitUntil(shootTrigger)
                                .andThen(Commands.race(
                                                NamedCommands.getCommand("ShootCommand"),
                                                Commands.waitSeconds(2.0)
                                                        .andThen(NamedCommands.getCommand("ShakeDeployCommand")))
                                        .withTimeout(5.0)
                                        .asProxy())),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))),
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(2)),
                        Commands.waitUntil(shootTrigger).andThen(NamedCommands.getCommand("ShootCommand"))));
    }
}
