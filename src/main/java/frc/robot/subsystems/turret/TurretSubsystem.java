package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    private final PIDController controller;
    private Angle targetAngle = kMinimumAngle;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.robotPoseSupplier = robotPoseSupplier;

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kAngleTolerance.in(Degrees));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public Command updateControl() {
        return run(() -> {
            targetAngle = Degrees.of(
                    MathUtil.clamp(targetAngle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));

            double output = controller.calculate(inputs.angle.in(Degrees), targetAngle.in(Degrees));
            output = MathUtil.clamp(output, -12.0, 12.0);

            io.setVoltage((Volts.of(output)));
        });
    }

    public Command setTargetAngle(Angle angle) {
        return runOnce(() -> targetAngle = angle);
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    @AutoLogOutput(key = "Turret/TargetAngle")
    public Angle getTargetAngle() {
        return targetAngle;
    }

    @AutoLogOutput(key = "Turret/AtTargetAngle")
    public boolean atTargetAngle() {
        return controller.atSetpoint();
    }

    public Command stop() {
        return runOnce(() -> {
            controller.reset();
            io.stop();
        });
    }

    public Command setAngleToHubCommand() {
        return run(() -> {
            Angle safe = calculateSafeAngle(getCurrentAngle(), calculateHubAngle());
            setTargetAngle(safe);
        });
    }

    private Angle calculateHubAngle() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = FieldUtil.getTargetHubPose();

        Distance dx = targetPose.getMeasureX().minus(robotPose.getMeasureY());
        Distance dy = targetPose.getMeasureY().minus(robotPose.getMeasureY());

        double fieldAngle = Math.atan2(dy.in(Meters), dx.in(Meters));
        double robotYaw = robotPose.getRotation().getRadians();
        double turretAngle = fieldAngle - robotYaw;

        return Radians.of(turretAngle);
    }

    private Angle calculateSafeAngle(Angle current, Angle desired) {
        Angle safeRange = kMaximumAngle.minus(kMinimumAngle);
        Angle errorRange = desired.minus(current);

        Angle error = Degrees.of(
                MathUtil.inputModulus(errorRange.in(Degrees), -safeRange.in(Degrees), safeRange.in(Degrees)));
        Angle candidate = current.plus(error);

        return Degrees.of(MathUtil.clamp(candidate.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
    }
}
