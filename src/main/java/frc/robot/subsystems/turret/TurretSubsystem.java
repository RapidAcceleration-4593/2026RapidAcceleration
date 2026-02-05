package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    private Angle targetAngle = kMinimumAngle;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.poseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        targetAngle = inputs.targetAngle;

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Command", "none");
        }
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public boolean atTargetAngle() {
        return inputs.angle.isNear(targetAngle, kAngleTolerance);
    }

    /**
     * Constructs a command to run the turret at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop)
                .until(() -> (inputs.angle.lt(kMinimumAngle) || inputs.angle.lt(kMaximumAngle)));
    }

    /**
     * Constructs a command to run the turret to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return startEnd(() -> setPosition(angle), io::stop).until(this::atTargetAngle);
    }

    /**
     * Constructs a command to continuously run the turret to the calculated Hub angle.
     *
     * @return A command to run the motor to the calculated Hub angle without stopping.
     */
    public Command pointAtHubCommand() {
        return runEnd(
                () -> setPosition(calculateSafeAngle(
                        getCurrentAngle(), calculateHubAngle().get())),
                io::stop);
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
     * Calculates the angle based on the robot's rotation from the hub.
     *
     * @return An angle supplier from a linear regression equation.
     */
    private Supplier<Angle> calculateHubAngle() {
        return () -> {
            Pose2d robotPose = poseSupplier.get();
            Pose2d targetPose = FieldUtil.getTargetHubPose();

            Distance dx = targetPose.getMeasureX().minus(robotPose.getMeasureY());
            Distance dy = targetPose.getMeasureY().minus(robotPose.getMeasureY());

            double fieldAngle = Math.atan2(dy.in(Meters), dx.in(Meters));
            double robotYaw = robotPose.getRotation().getRadians();
            double turretAngle = fieldAngle - robotYaw;

            return Radians.of(turretAngle);
        };
    }

    /** Sets the angle of the closed-loop PID control. */
    private void setPosition(Angle angle) {
        Angle clampedAngle =
                Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }

    /** Calculates a safe angle based on the soft wrapping limits. */
    private Angle calculateSafeAngle(Angle current, Angle desired) {
        Angle safeRange = kMaximumAngle.minus(kMinimumAngle);
        Angle errorRange = desired.minus(current);

        Angle error = Degrees.of(
                MathUtil.inputModulus(errorRange.in(Degrees), -safeRange.in(Degrees), safeRange.in(Degrees)));
        Angle candidate = current.plus(error);

        return Degrees.of(MathUtil.clamp(candidate.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
    }
}
