package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShotCalculatorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {

    ShotCalculatorSubsystem calculator;

    public ShootCommand(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            IndexerSubsystem indexer,
            ShotCalculatorSubsystem calculator,
            Pose3d targetPose3d) {

        BooleanSupplier readySupplier = () -> shooter.atTargetVelocity() && hood.atTargetAngle();
        addCommands(
                Commands.runOnce(() -> calculator.setTarget(targetPose3d)),
                shooter.runAtVelocityCommand(calculator::getShooterVelocity),
                hood.runToAngleCommand(calculator::getHoodAngle),
                indexer.runCommand().onlyWhile(readySupplier).repeatedly());
    }
}
