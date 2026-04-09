package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fAngleMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.AngleMechanism3D;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final HoodIO io;
    private final HoodInputsAutoLogged inputs;
    private final AngleMechanism3D hood3D;

    private Angle targetAngle = kMinimumAngle;

    public HoodSubsystem(HoodIO io) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();

        Trigger lsTrigger = new Trigger(() -> inputs.bottomLS);
        lsTrigger.onTrue(Commands.runOnce(io::resetPosition));
        hood3D = fAngleMechanism3D.find("Hood");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        hood3D.setAngle(inputs.angle);
        CommandLogger.logSubsystemCommand(this);
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    public Angle getTargetAngle() {
        return inputs.targetAngle;
    }

    public boolean atTargetAngle() {
        return inputs.angle.isNear(targetAngle, kAngleTolerance);
    }

    /**
     * Constructs a command to run the hood at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return runEnd(() -> setVoltage(volts), io::stop);
    }

    /**
     * Constructs a command to run the hood to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return startEnd(() -> setPosition(() -> angle), io::stop).until(this::atTargetAngle);
    }

    /**
     * Constructs a command to run the hood continuously to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle without stopping.
     */
    public Command runToAngleCommand(Supplier<Angle> angle) {
        return runEnd(() -> setPosition(angle), () -> setPosition(() -> kMinimumAngle));
    }

    /**
     * Constructs a command to stop the hood motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    private void setVoltage(Voltage volts) {
        if (inputs.bottomLS && volts.lt(Volts.zero())) {
            io.stop();
        } else {
            io.setVoltage(volts);
        }
    }

    /**
     * Sets the angle of the closed-loop PID controller.
     *
     * @param angleSupplier The supplied angle to set as the hood position.
     */
    private void setPosition(Supplier<Angle> angleSupplier) {
        Angle angle = angleSupplier.get();
        Angle clampedAngle =
                Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        this.targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }
}
