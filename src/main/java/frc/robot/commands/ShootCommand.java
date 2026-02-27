package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.shooting.ShotCalculator;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {

    /** Points the hood at the hub, spins up the shooter, and runs the indexer until this Command is canceled. */
    public ShootCommand(
            ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer, ShotCalculator calculator) {
        BooleanSupplier readySupplier =
                () -> shooter.atTargetVelocity() && hood.atTargetAngle(); // && turret.atTargetAngle()
        addCommands(
                shooter.runAtVelocityCommand(() -> RPM.of(SmartDashboard.getNumber("ShootRPM", 4000))),
                hood.runToAngleCommand(() -> Degrees.of(SmartDashboard.getNumber("HoodDegrees", 20))),
                indexer.runCommand().onlyWhile(readySupplier).repeatedly(),
                Commands.run(() -> calculator.calculate()));
    }
}
