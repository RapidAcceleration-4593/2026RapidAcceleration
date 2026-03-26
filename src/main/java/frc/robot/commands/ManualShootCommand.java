package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ManualShootCommand extends ParallelCommandGroup {

    public ManualShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {

        addCommands(
                shooter.setVoltageCommand(Volts.of(7.5)),
                hood.goToAngleCommand(HoodConstants.kMinimumAngle),
                Commands.waitSeconds(0.4).andThen(indexer.runCommand()));
    }
}
