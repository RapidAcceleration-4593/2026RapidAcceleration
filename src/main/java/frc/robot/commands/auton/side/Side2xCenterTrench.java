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
        EventTrigger shootTrigger = new EventTrigger("Shoot");

        addCommands(
    // === PATH 1 + DEPLOY/INTAKE ===
    Commands.parallel(
        AutoBuilder.followPath(paths.get(0)),

        Commands.sequence(
            NamedCommands.getCommand("ExtendDeployCommand"),
            NamedCommands.getCommand("RetractDeployCommand"),
            NamedCommands.getCommand("IntakeCommand")
        ),

        // === SHOOT SEQUENCE (fully self-contained, no interference) ===
        Commands.sequence(
            Commands.waitUntil(shootTrigger),

            // Step 1: start shooting
            Commands.runOnce(() -> {
                NamedCommands.getCommand("ShootCommand").schedule();
            }),

            // Step 2: wait 2 seconds (shooting only)
            Commands.waitSeconds(2.0),

            // Step 3: start shaking (independent)
            Commands.runOnce(() -> {
                NamedCommands.getCommand("ShakeDeployCommand").schedule();
            }),

            // Step 4: wait remaining 3 seconds
            Commands.waitSeconds(3.0),

            // Step 5: FORCE STOP BOTH (this is what you were missing)
            Commands.runOnce(() -> {
                NamedCommands.getCommand("ShootCommand").cancel();
                NamedCommands.getCommand("ShakeDeployCommand").cancel();
            })
        )
    ),

    // === PATH 2 ===
    Commands.parallel(
        AutoBuilder.followPath(paths.get(1)),
        NamedCommands.getCommand("IntakeCommand")
    ),

    // === PATH 3 + FINAL SHOOT ===
    Commands.parallel(
        AutoBuilder.followPath(paths.get(2)),

        Commands.sequence(
            Commands.waitUntil(shootTrigger),

            Commands.runOnce(() -> {
                NamedCommands.getCommand("ShootCommand").schedule();
            }),

            Commands.waitSeconds(5.0),

            Commands.runOnce(() -> {
                NamedCommands.getCommand("ShootCommand").cancel();
            })
        )
    )
);
    }
}
