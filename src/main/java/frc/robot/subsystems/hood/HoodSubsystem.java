package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HoodSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d hood;

    private Angle targetAngle = kMinimumAngle;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("HoodRoot", 0.5, 0.5);
        hood = root.append(new LoggedMechanismLigament2d("Hood", Inches.of(12), kMinimumAngle));

        Trigger limitSwitchTrigger = new Trigger(() -> inputs.limitswitch);
        limitSwitchTrigger.onTrue(Commands.runOnce(io::resetPosition));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        targetAngle = inputs.targetAngle;

        hood.setAngle(inputs.angle);
        Logger.recordOutput("Mechanisms/Hood", mechanism);
        if (getCurrentCommand() != null)
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        else Logger.recordOutput("Command", "none");
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
        return Commands.runOnce(() -> io.setVoltage(volts)).finallyDo(io::stop);
    }

    /**
     * Constructs a command to run the hood to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return run(() -> this.setPosition(angle)).until(this::atTargetAngle).finallyDo(io::stop);
    }

    /**
     * Constructs a command to continuously run the hood to the calculated Hub angle.
     *
     * @return A command to run the motor to the calculated Hub angle without stopping.
     */
    public Command pointAtHubCommand() {
        return run(() -> this.setPosition(calculateHubAngle().get())).finallyDo(io::stop);
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
        Pose2d targetPose = FieldUtil.getTargetHubPose();
        Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));
        return () -> Degrees.of(10.0 * distance.in(Meters));
    }

    /** Sets the angle of the closed-loop PID control. */
    private void setPosition(Angle angle) {
		Angle clampedAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
    	targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }
}
