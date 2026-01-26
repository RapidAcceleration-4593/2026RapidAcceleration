package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command setVelocity(AngularVelocity velocity) {
        return runOnce(() -> io.setVelocity(velocity));
    }

    public AngularVelocity getCurrentVelocity() {
        return inputs.velocity;
    }

    public AngularVelocity getTargetVelocity() {
        return inputs.targetVelocity;
    }

    public boolean atTargetVelocity() {
        return inputs.atTargetVelocity;
    }

    public Command stop() {
        return runOnce(io::stop);
    }
}
