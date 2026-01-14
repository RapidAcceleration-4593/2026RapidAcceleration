package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    private final PIDController pidController;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.robotPoseSupplier = robotPoseSupplier;
        this.pidController = kTurretPID;

        pidController.enableContinuousInput(kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees));
        pidController.setTolerance(kMaximumTolerance.in(Degrees));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double currentAngle = getAngle();
        double desiredAngle = calculateDesiredAngle();
        double safeSetpoint = wrapToSafeRange(currentAngle, desiredAngle);

        double output = pidController.calculate(currentAngle, safeSetpoint);

        setMotorSpeed(output);
    }

    public void setMotorSpeed(double speed) {
        io.setMotorSpeed(speed);
    }

    public void stopMotor() {
        io.stopMotor();
    }

    public double getAngle() {
        return inputs.angle.in(Degrees);
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    private double calculateDesiredAngle() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = kTargetPose;

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double robotYaw = robotPose.getRotation().getRadians();
        double turretAngle = fieldAngle - robotYaw;

        return Math.toDegrees(turretAngle);
    }

    private double wrapToSafeRange(double currentAngle, double desiredAngle) {
        double min = kMinimumAngle.in(Degrees);
        double max = kMaximumAngle.in(Degrees);

        // Choose the shortest path according to error.
        double error = MathUtil.inputModulus(desiredAngle - currentAngle, min - max, max - min);
        double candidate = currentAngle + error;

        return MathUtil.clamp(candidate, min, max);
    }
}
