package frc.robot;

import static frc.robot.Constants.Controllers.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // Subsystem(s)
    // Initialize IntakeSubsystem.

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(operatorControllerPort);

    public RobotContainer() {
        registerCommands();
        configureBindings();
    }

    private void configureBindings() {}
    private void registerCommands() {}

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
