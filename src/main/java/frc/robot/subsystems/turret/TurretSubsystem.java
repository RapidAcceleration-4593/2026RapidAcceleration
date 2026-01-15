package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    private final PIDController pidController;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.robotPoseSupplier = robotPoseSupplier;

        this.pidController = new PIDController(kTurretP, kTurretI, kTurretD);
        pidController.setTolerance(kMaximumTolerance.in(Degrees));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double currentAngle = getAngle();
        double desiredAngle = calculateDesiredAngle();
        double safeSetpoint = wrapToSafeRange(currentAngle, desiredAngle);

        double output = pidController.calculate(currentAngle, safeSetpoint);
        Logger.recordOutput("Turret/Setpoint", safeSetpoint); // Switch to inputs.angle.

        setMotorSpeed(output);

        if (kCurrentMode == Mode.SIM) {
            Rotation3d turretRotation = new Rotation3d(0.0, 0.0, Units.degreesToRadians(safeSetpoint));
            Pose3d turretPose = new Pose3d(0.0, 0.0, 0.0, turretRotation);

            Logger.recordOutput("Turret/Pose", turretPose);
        }
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
        Pose2d targetPose = kAlliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;

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
