package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class DeploySubsystem extends SubsystemBase {

    private final DeployInputsAutoLogged inputs;
    private final DeployIO io;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d deploy;

    private Distance targetDistance = kMinimumDistance;

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("DeployRoot", 0.5, 0.5);
        deploy = root.append(new LoggedMechanismLigament2d("Deploy", Inches.of(12), Degrees.zero()));

        Trigger lsTrigger = new Trigger(() -> (inputs.inLimitSwitch || inputs.outLimitSwitch));
        lsTrigger.onTrue(Commands.runOnce(io::resetPosition));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);
        targetDistance = inputs.targetDistance;

        deploy.setLength(inputs.distance);
        Logger.recordOutput("Mechanisms/Deploy", mechanism);
        if (getCurrentCommand() != null) {
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Command", "none");
        }
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    public Distance getTargetDistance() {
        return inputs.targetDistance;
    }

    public boolean atTargetDistance() {
        return inputs.distance.isNear(targetDistance, kDistanceTolerance);
    }

    /**
     * Constructs a command to run the deploy at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop);
    }

    /**
     * Constructs a command to run the deploy to a set distance.
     *
     * @param distance The distance to apply to the closed-loop PID control.
     * @return A command to run the motor to a distance and stop when complete.
     */
    public Command goToDistanceCommand(Distance distance) {
        return startEnd(() -> setPosition(distance), io::stop).until(this::atTargetDistance);
    }

    /**
     * Constructs a command to stop the deploy motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /** Sets the distance of the closed-loop PID control. */
    private void setPosition(Distance distance) {
        this.targetDistance = distance;
        io.setPosition(distance);
    }
}
