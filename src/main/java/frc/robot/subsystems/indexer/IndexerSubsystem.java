package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fWheelMechanism3D;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.WheelMechanism3D;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final IndexerInputsAutoLogged inputs;
    private final IndexerIO io;

    private final WheelMechanism3D indexer3D;

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
        this.inputs = new IndexerInputsAutoLogged();
        indexer3D = fWheelMechanism3D.find("Indexer");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        indexer3D.setAngularVelocity(inputs.spindexerVelocity);

        CommandLogger.logSubsystemCommand(this);
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
