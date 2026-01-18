package frc.robot.subsystems.indexer;

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

    public void setSpindexerSpeed(double speed) {
        io.setSpindexerSpeed(speed);
    }

    public void stopSpindexer() {
        io.stopSpindexer();
    }

    public void setFeederSpeed(double speed) {
        io.setFeederSpeed(speed);
    }

    public void stopFeeder() {
        io.stopFeeder();
    }
}
