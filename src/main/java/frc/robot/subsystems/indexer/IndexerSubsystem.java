package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.kFeederVolts;
import static frc.robot.subsystems.indexer.IndexerConstants.kSpindexerVolts;

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
    }

    public Command run() {
        return runOnce(() -> {
            io.setSpindexerVoltage(kSpindexerVolts);
            io.setFeederVoltage(kFeederVolts);
        });
    }

    public Command stop() {
        return runOnce(io::stop);
    }
}
