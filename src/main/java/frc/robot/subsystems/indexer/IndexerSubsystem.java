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

    /**
     * Constructs a command to only run the spindexer at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the spindexer motor and stop when complete.
     */
    public Command setSpindexerVoltageCommand(Voltage volts) {
        return Commands.runOnce(() -> io.setSpindexerVoltage(volts)).finallyDo(io::stopSpindexer);
    }

    /**
     * Constructs a command to only run the feeder at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the feeder motor and stop when complete.
     */
    public Command setFeederVoltageCommand(Voltage volts) {
        return Commands.runOnce(() -> io.setFeederVoltage(volts)).finallyDo(io::stopFeeder);
    }

    /**
     * Constructs a command to run both the spindexer and feeder motors.
     *
     * @return A command to run the spindexer and feeder motors and stop when complete.
     */
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
