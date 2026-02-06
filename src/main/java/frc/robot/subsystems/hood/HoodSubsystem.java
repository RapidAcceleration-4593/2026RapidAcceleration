package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fAngleMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FieldUtil;
import frc.robot.util.mechanism.AngleMechanism3D;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    private Angle targetAngle = kMinimumAngle;

    private final AngleMechanism3D hood3D;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        Trigger limitSwitchTrigger = new Trigger(() -> inputs.limitswitch);
        limitSwitchTrigger.onTrue(Commands.runOnce(io::resetPosition));
        hood3D = fAngleMechanism3D.find("Hood");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        targetAngle = inputs.targetAngle;

        hood3D.setAngle(inputs.angle);
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
        return startEnd(() -> io.setVoltage(volts), io::stop)
                .until(() -> (inputs.limitswitch && inputs.appliedVolts.lt(Volts.zero())));
    }

    /**
     * Constructs a command to run the hood to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return startEnd(() -> setPosition(angle), io::stop).until(this::atTargetAngle);
    }

    /**
     * Constructs a command to continuously run the hood to the calculated Hub angle.
     *
     * @return A command to run the motor to the calculated Hub angle without stopping.
     */
    public Command pointAtHubCommand() {
        return runEnd(() -> setPosition(calculateHubAngle().get()), io::stop);
    }

    /**
     * Constructs a command to stop the hood motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /**
     * Calculates the angle based on the distance from the Hub.
     *
     * @return An angle supplier from a linear regression equation.
     */
    private Supplier<Angle> calculateHubAngle() {
        return () -> {
            Pose2d targetPose = FieldUtil.getTargetHubPose();
            Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

            Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));
            return Degrees.of(10.0 * distance.in(Meters));
        };
    }

    /** Sets the angle of the closed-loop PID control. */
    private void setPosition(Angle angle) {
        Angle clampedAngle =
                Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }
}
