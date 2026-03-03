package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.shooting.ShotCalculator;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {
	ShotCalculator calculator;

    /** Points the hood at the hub, spins up the shooter, and runs the indexer until this Command is canceled. */
    public ShootCommand(
            ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer, ShotCalculator calculator, Pose3d targetPose3d) {

        BooleanSupplier readySupplier = () -> shooter.atTargetVelocity() && hood.atTargetAngle();
        addCommands(
				Commands.run(() -> calculator.calculate(targetPose3d)),
                shooter.runAtVelocityCommand(calculator::getShooterVelocity),
                hood.runToAngleCommand(calculator::getHoodAngle),
                indexer.runCommand().onlyWhile(readySupplier).repeatedly());
    }
}
