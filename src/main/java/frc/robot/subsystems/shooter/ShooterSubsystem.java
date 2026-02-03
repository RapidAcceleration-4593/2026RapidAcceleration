package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterInputsAutoLogged inputs;
    private final ShooterIO io;

    private AngularVelocity targetVelocity = kZeroVelocity;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        targetVelocity = inputs.targetVelocity;

        if (getCurrentCommand() != null)
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        else Logger.recordOutput("Command", "none");
    }

    public AngularVelocity getCurrentVelocity() {
        return inputs.velocity;
    }

    public AngularVelocity getTargetVelocity() {
        return inputs.targetVelocity;
    }

    public boolean atTargetVelocity() {
        return inputs.velocity.isNear(targetVelocity, kVelocityTolerance);
    }

    public Command runAtVelocityCommand(AngularVelocity velocity) {
        return run(() -> this.setVelocity(velocity)).finallyDo(io::stop);
    }

    public Command setVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setVoltage(volts)).finallyDo(io::stop);
    }

    public Command stopCommand() {
        return runOnce(io::stop);
    }

    private void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        io.setVelocity(velocity);
    }
}
