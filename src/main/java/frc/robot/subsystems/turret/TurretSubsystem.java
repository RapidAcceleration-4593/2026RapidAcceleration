package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();

        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateControl();
        Logger.processInputs("Turret", inputs);
    }

    public void setAngle(Angle angle) {
        io.setAngle(angle);
    }

    public Angle getAngle() {
        return inputs.angle;
    }

    public boolean atAngle() {
        return inputs.atAngle;
    }

    public void stop() {
        io.stop();
    }

    public double calculateDesiredAngle() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = kAlliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double robotYaw = robotPose.getRotation().getRadians();
        double turretAngle = fieldAngle - robotYaw;

        return Math.toDegrees(turretAngle);
    }

    public double wrapToSafeRange(double currentAngle, double desiredAngle) {
        double min = kMinimumAngle.in(Degrees);
        double max = kMaximumAngle.in(Degrees);

        // Choose the shortest path according to error.
        double error = MathUtil.inputModulus(desiredAngle - currentAngle, min - max, max - min);
        double candidate = currentAngle + error;

        return MathUtil.clamp(candidate, min, max);
    }
}
