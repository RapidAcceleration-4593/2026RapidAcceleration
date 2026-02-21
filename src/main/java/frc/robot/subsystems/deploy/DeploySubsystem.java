package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fLengthMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.LengthMechanism3D;
import org.littletonrobotics.junction.Logger;

public class DeploySubsystem extends SubsystemBase {

    private final DeployInputsAutoLogged inputs;
    private final DeployIO io;

    private Distance targetDistance = kMinimumDistance;

    private final LengthMechanism3D deploy3D;

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();

        Trigger lsTrigger = new Trigger(() -> inputs.retractedLS);
        lsTrigger.onTrue(Commands.runOnce(io::resetPosition));

        deploy3D = fLengthMechanism3D.find("Deploy");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);
        targetDistance = inputs.targetDistance;

        deploy3D.setLength(inputs.distance);
        CommandLogger.logSubsystemCommand(this);

        if (inputs.retractedLS) {
            io.resetPosition();
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
        return startEnd(() -> setVoltage(volts), io::stop);
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

    /**
     * Sets the distance of the closed-loop PID controller.
     *
     * @param distance The distance to set as the deploy position.
     */
    private void setPosition(Distance distance) {
        if (inputs.retractedLS && distance.lte(kMinimumDistance)) {
            distance = kMinimumDistance;
        }

        Distance clampedDistance = Inches.of(
                MathUtil.clamp(distance.in(Inches), kMinimumDistance.in(Inches), kMaximumDistance.in(Inches)));
        targetDistance = clampedDistance;
        io.setPosition(distance);
    }

    /**
     * Sets the voltage of the motor.
     *
     * @param volts The voltage to apply to the hood motor.
     */
    private void setVoltage(Voltage volts) {
        if (inputs.retractedLS && volts.lt(Volts.zero())) {
            io.stop();
        } else {
            io.setVoltage(volts);
        }
    }
}
