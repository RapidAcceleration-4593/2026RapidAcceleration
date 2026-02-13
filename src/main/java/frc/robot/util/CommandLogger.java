package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class CommandLogger {

    public static void logSubsystemCommand(SubsystemBase subsystem) {
        if (subsystem.getCurrentCommand() != null) {
            Logger.recordOutput(
                    "Subsystems/" + subsystem.getName() + "/CurrentCommand",
                    subsystem.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Subsystems/" + subsystem.getName() + "/CurrentCommand", "none");
        }
    }
}
