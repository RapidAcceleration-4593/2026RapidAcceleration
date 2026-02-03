package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /**
     * Constructs a command that continuously attempts to reach the supplied angle.
     *
     * @param angleSupplier The function that supplies the setpoint angle this subsystem should reach.
     * @param finishWhenTargetReached Should the command finish when the target is reached?
     */
    public Command goToAngleCommand(Supplier<Angle> angleSupplier, boolean finishWhenTargetReached) {
        Command command = run(() -> {
                    this.targetAngle = angleSupplier.get();
                    io.setPosition(angleSupplier.get());
                })
                .finallyDo(io::stop);
        return finishWhenTargetReached ? command.until(this::atTargetAngle) : command;
    }

    /**
     * Constructs a command that attempts to reach the given angle, and finishes when the angle has been reached.
     *
     * @param angle The angle this subsystem should reach.
     */
    public Command goToAngleCommand(Angle angle) {
        return goToAngleCommand(() -> angle, true);
    }

    /** Constructs a command that continuously adjusts the hood for the hub based on the current robot position. */
    public Command pointAtHubCommand() {
        return goToAngleCommand(this::calculateHubAngle, false);
    }

    /** Calculates the optimal hood angle to shoot at the hub based on the current robot position. */
    private Angle calculateHubAngle() {
        Pose2d targetPose = FieldUtil.getTargetHubPose();
        Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));
        return Degrees.of(8.0 * distance.in(Meters));
    }
}
