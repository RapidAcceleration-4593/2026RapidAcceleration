package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class CommandLogger {

    public static void logSubsystemCommand(SubsystemBase subsystem) {
        Command command = subsystem.getCurrentCommand();
        if (command != null) {
            Logger.recordOutput("Subsystems/" + subsystem.getName() + "/CurrentCommand", command.getName());
        } else {
            Logger.recordOutput("Subsystems/" + subsystem.getName() + "/CurrentCommand", "none");
        }
    }
}
