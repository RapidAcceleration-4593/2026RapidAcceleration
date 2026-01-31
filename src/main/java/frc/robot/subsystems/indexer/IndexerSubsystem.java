package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fWheelMechanism3D;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }

    public Command runCommand() {
        return runOnce(() -> {
            io.setSpindexerVoltage(kSpindexerVolts);
            io.setFeederVoltage(kFeederVolts);
        });
    }

    public Command stopCommand() {
        return runOnce(io::stop);
    }
}
