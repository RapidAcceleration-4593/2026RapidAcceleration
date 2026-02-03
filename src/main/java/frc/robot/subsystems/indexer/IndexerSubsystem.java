package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    }

    public Command setSpindexerVoltageCommand(Voltage volts) {
        return Commands.runOnce(() -> io.setSpindexerVoltage(volts)).finallyDo(io::stopSpindexer);
    }

    public Command setFeederVoltageCommand(Voltage volts) {
        return Commands.runOnce(() -> io.setFeederVoltage(volts)).finallyDo(io::stopFeeder);
    }

    public Command runCommand() {
        return runOnce(() -> {
                    io.setSpindexerVoltage(kSpindexerVolts);
                    io.setFeederVoltage(kFeederVolts);
                })
                .finallyDo(() -> {
                    io.stopSpindexer();
                    io.stopFeeder();
                });
    }

    public Command stopCommand() {
        return runOnce(() -> {
            io.stopSpindexer();
            io.stopFeeder();
        });
    }
}
