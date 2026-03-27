package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideCenterLoop extends AutonCommand {

    public SideCenterLoop(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterLoop-1"));

        addCommands(
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(0)),
                        Commands.waitSeconds(1.0).andThen(NamedCommands.getCommand("IntakeCommand"))),
                NamedCommands.getCommand("ShakeDeployCommand"));
    }
}
