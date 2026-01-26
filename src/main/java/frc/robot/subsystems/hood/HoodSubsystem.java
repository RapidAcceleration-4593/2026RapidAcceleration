package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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

    private final PIDController controller;
    private Angle targetAngle = kMinimumAngle;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("HoodRoot", 0.5, 0.5);
        hood = root.append(new LoggedMechanismLigament2d("Hood", Inches.of(12), kMinimumAngle));

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kAngleTolerance.in(Degrees));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        hood.setAngle(inputs.angle);
        Logger.recordOutput("Mechanisms/Hood", mechanism);
    }

    public Command updateControl() {
        return run(() -> {
            targetAngle = Degrees.of(
                    MathUtil.clamp(targetAngle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));

            double output = controller.calculate(inputs.angle.in(Degrees), targetAngle.in(Degrees));
            output = MathUtil.clamp(output, -12.0, 12.0);

            io.setVoltage(Volts.of(output));
        });
    }

    public Command setTargetAngle(Angle angle) {
        return runOnce(() -> targetAngle = angle);
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    @AutoLogOutput(key = "Hood/TargetAngle")
    public Angle getTargetAngle() {
        return targetAngle;
    }

    @AutoLogOutput(key = "Hood/AtTargetAngle")
    public boolean atTargetAngle() {
        return controller.atSetpoint();
    }

    public Command stop() {
        return runOnce(() -> {
            controller.reset();
            io.stop();
        });
    }

    public Command setAngleCommand(Angle angle) {
        return runOnce(() -> setTargetAngle(angle));
    }

    public Command setAngleToHubCommand() {
        return run(() -> setTargetAngle(calculateHubAngle()));
    }

    private Angle calculateHubAngle() {
        Pose2d targetPose =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? kBlueHubPose : kRedHubPose;
        Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));

        return Degrees.of(8.0 * distance.in(Meters));
    }
}
