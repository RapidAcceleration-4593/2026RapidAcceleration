package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterInputsAutoLogged inputs;
    private final ShooterIO io;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    public boolean atVelocity() {
        return inputs.atTargetVelocity;
    }

    public void stop() {
        io.stop();
    }
}
