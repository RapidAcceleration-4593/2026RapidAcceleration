package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.AngleMechanism3D;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final TurretIO io;
    private final TurretInputsAutoLogged inputs;

    private Angle targetAngle = kInitialAngle;
    private AngleMechanism3D turret3D;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.turret3D = fAngleMechanism3D.find("Turret");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        turret3D.setAngle(inputs.angle);

        CommandLogger.logSubsystemCommand(this);
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    public Angle getTargetAngle() {
        return inputs.targetAngle;
    }

    public boolean atTargetAngle() {
        // Should be if the current angle is within tolerance of the true target angle, not the safe wrapped angle.
        return inputs.angle.isNear(targetAngle, kAngleTolerance);
    }

    /**
     * Constructs a command to run the turret at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop);
    }

    /**
     * Constructs a command to run the turret to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return startEnd(() -> setPosition(() -> angle), io::stop).until(this::atTargetAngle);
    }

    /**
     * Constructs a command to run the turret continuously to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle without stopping.
     */
    public Command runToAngleCommand(Supplier<Angle> angleSupplier) {
        return runEnd(() -> setPosition(() -> calculateSafeAngle(angleSupplier.get())), io::stop);
    }

    /**
     * Constructs a command to stop the turret motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /**
     * Calculates a safe angle to run the turret to based on the current and desired angles.
     *
     * @return A safe angle that does not exceed the physical limits of the turret.
     */
    private Angle calculateSafeAngle(Angle targetAngle) {
        Angle current = getCurrentAngle();
        Angle error =
                Degrees.of(MathUtil.inputModulus(targetAngle.minus(current).in(Degrees), -180.0, 180.0));
        Angle candidate = current.plus(error);

        if (candidate.lt(kMinimumAngle.minus(kWrapMargin))) {
            candidate = candidate.plus(Degrees.of(360.0));
        } else if (candidate.gt(kMaximumAngle.plus(kWrapMargin))) {
            candidate = candidate.minus(Degrees.of(360.0));
        }
        return Degrees.of(MathUtil.clamp(candidate.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
    }

    /**
     * Sets the angle of the closed-loop PID controller.
     *
     * @param angleSupplier The supplied angle to set as the turret position.
     */
    private void setPosition(Supplier<Angle> angleSupplier) {
        Angle angle = angleSupplier.get();
        Angle clampedAngle =
                Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        this.targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }
}
