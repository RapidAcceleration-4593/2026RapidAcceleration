package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs;

    private AngularVelocity targetVelocity = RPM.zero();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        CommandLogger.logSubsystemCommand(this);
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

    /**
     * Constructs a command to run the shooter at a set voltage. ONLY FOR EXPERIMENTAL USE.
     *
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop);
    }

    /**
     * Constructs a command to run the shooter at a set velocity.
     *
     * @param velocity The velocity to apply to the closed-loop PID control.
     * @return A command to run the motor at a velocity and stop when completed.
     */
    public Command runAtVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return runEnd(() -> setVelocity(velocitySupplier), io::stop);
    }

    /**
     * Constructs a command to stop the shooter motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /**
     * Sets the velocity of the closed-loop feedforward controller.
     *
     * @param velocity The velocity to set as the shooter velocity.
     */
    private void setVelocity(Supplier<AngularVelocity> velocitySupplier) {
        AngularVelocity velocity = velocitySupplier.get();
        this.targetVelocity = velocity;
        io.setVelocity(velocity);
    }
}
