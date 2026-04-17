package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.deploy.DeployConstants.kMaximumDistance;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeCommand extends ParallelCommandGroup {

    public OuttakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                deploy.goToDistanceCommand(kMaximumDistance, false).withTimeout(2.0),
                intake.setVoltageCommand(Volts.of(-12.0)));
    }
}
