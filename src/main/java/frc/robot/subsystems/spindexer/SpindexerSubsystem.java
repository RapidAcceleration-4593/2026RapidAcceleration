package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {

    private final SpindexerInputsAutoLogged inputs;
    private final SpindexerIO io;

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;
        this.inputs = new SpindexerInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
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
