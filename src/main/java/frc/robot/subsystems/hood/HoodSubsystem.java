package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    public HoodSubsystem(HoodIO io) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateControl();
        Logger.processInputs("Hood", inputs);
    }

    public void setAngle(double degrees) {
        io.setAngle(degrees);
    }

    public double getAngle() {
        return io.getAngle();
    }

    public boolean atAngle() {
        return io.atAngle();
    }

    public void stop() {
        io.stop();
    }
}
