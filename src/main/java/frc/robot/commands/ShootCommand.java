package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShotCalculatorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            IndexerSubsystem indexer,
            ShotCalculatorSubsystem calculator) {

        addCommands(
                shooter.runAtVelocityCommand(calculator::getShooterVelocity),
                hood.runToAngleCommand(calculator::getHoodAngle),
                Commands.sequence(Commands.waitSeconds(0.5), indexer.runCommand()));
    }
}
