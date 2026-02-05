package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
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

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Command", "none");
        }
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
    public Command runCommand() {
        return startEnd(() -> io.setVoltage(kShooterVolts), io::stop);
    }

    /**
     * Constructs a command to run the shooter at a set velocity.
     *
     * @param velocity The velocity to apply to the closed-loop PID control.
     * @return A command to run the motor at a velocity and stop when completed.
     */
    public Command runAtVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    /**
     * Constructs a command to stop the shooter motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /** Sets the velocity of the closed-loop PID control. */
    private void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        io.setVelocity(velocity);
    }
}
