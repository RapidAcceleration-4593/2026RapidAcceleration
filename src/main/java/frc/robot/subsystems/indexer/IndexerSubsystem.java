package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final IndexerInputsAutoLogged inputs;
    private final IndexerIO io;

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
        this.inputs = new IndexerInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Command", "none");
        }
    }

    /**
     * Constructs a command to run both the spindexer and feeder motors.
     *
     * @return A command to run the spindexer and feeder motors and stop when complete.
     */
    public Command runCommand() {
        return startEnd(
                () -> {
                    io.setSpindexerVoltage(kSpindexerVolts);
                    io.setFeederVoltage(kFeederVolts);
                },
                () -> {
                    io.stopSpindexer();
                    io.stopFeeder();
                });
    }

    /**
     * Constructs a command to stop both the spindexer and feeder motors.
     *
     * @return A command to stop the motors immediately.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            io.stopSpindexer();
            io.stopFeeder();
        });
    }
}
