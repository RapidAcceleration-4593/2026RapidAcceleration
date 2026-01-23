package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeInputsAutoLogged inputs;
    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void run() {
        io.run();
    }

    public void stop() {
        io.stop();
    }
}
