package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideCenterBump extends AutonCommand {

    public SideCenterBump(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterBump-1"));
        EventTrigger shootTrigger = new EventTrigger("StartShooter");

        addCommands(Commands.parallel(
                Commands.race(
                        AutoBuilder.followPath(paths.get(0)),
                        NamedCommands.getCommand("ExtendDeployCommand")
                                .andThen(NamedCommands.getCommand("RetractDeployCommand"))
                                .andThen(NamedCommands.getCommand("IntakeCommand"))),
                Commands.waitUntil(shootTrigger)
                        .andThen(Commands.parallel(
                                NamedCommands.getCommand("ShootCommand"),
                                Commands.waitSeconds(6.0).andThen(NamedCommands.getCommand("ShakeDeployCommand"))))
                        .withTimeout(10.0)));
    }
}
