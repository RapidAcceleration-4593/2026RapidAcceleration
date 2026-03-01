package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ShotCalculatorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public ShotCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public ShotCalculation calculate() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);
        Pose2d targetPose = FieldUtil.getTargetHubPose().toPose2d();

        Distance horizontalDistance = FieldUtil.getDistanceToHub(robotPose);
        Distance verticalDistance = FieldUtil.getTargetHubPose().getMeasureZ().minus(kShooterHeight);

        Logger.recordOutput("ShooterDistance", horizontalDistance);

        Angle hoodAngle = calculateHood(horizontalDistance);
        LinearVelocity launchSpeed =
                ProjectilePhysics.calculateLaunchSpeed(hoodAngle, horizontalDistance, verticalDistance);

        Angle turretAngle = calculateTurret(robotPose, targetPose, launchSpeed, hoodAngle, horizontalDistance);
        AngularVelocity shooterVelocity = calculateShooter(launchSpeed, horizontalDistance, turretAngle);

        return new ShotCalculation(turretAngle, hoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(distance.in(Meters) * 3.57 + 7.145);
    }

    private AngularVelocity calculateShooter(LinearVelocity launchSpeed, Distance distance, Angle turretAngle) {
        // double kExitVelocityFactor = SmartDashboard.getNumber("ExitVelocityFactor", 0.35);
        double kExitVelocityFactor = ProjectilePhysics.getExitFactor(distance, turretAngle);
        return RadiansPerSecond.of(launchSpeed.in(MetersPerSecond) / (kWheelRadius.in(Meters) * kExitVelocityFactor));
    }

    private Angle calculateTurret(
            Pose2d robotPose,
            Pose2d targetPose,
            LinearVelocity launchSpeed,
            Angle hoodAngle,
            Distance horizontalDistance) {
        if (!FieldUtil.isInAllianceZone(robotPose)) {
            Angle fieldAngle = FieldUtil.getCurrentAlliance() == Alliance.Blue ? Degrees.of(180) : Degrees.zero();
            return robotPose.getRotation().getMeasure().minus(fieldAngle);
        }

        ChassisSpeeds robotRelative = chassisSpeedsSupplier.get();
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, robotPose.getRotation());

        Time tof = ProjectilePhysics.calculateTime(launchSpeed, hoodAngle, horizontalDistance);

        Distance robotDx = Meters.of(fieldRelative.vxMetersPerSecond).times(tof.in(Seconds));
        Distance robotDy = Meters.of(fieldRelative.vyMetersPerSecond).times(tof.in(Seconds));

        Distance predictedX = robotPose.getMeasureX().plus(robotDx);
        Distance predictedY = robotPose.getMeasureY().plus(robotDy);

        Distance dx = targetPose.getMeasureX().minus(predictedX);
        Distance dy = targetPose.getMeasureY().minus(predictedY);

        Angle fieldAngle = Radians.of(Math.atan2(dy.in(Meters), dx.in(Meters)));
        return robotPose.getRotation().getMeasure().minus(fieldAngle);
    }

    public Angle getHoodAngle() {
        return calculate().hoodAngle();
    }

    public AngularVelocity getShooterVelocity() {
        return calculate().shooterVelocity();
    }

    public Angle getTurretAngle() {
        return calculate().turretAngle();
    }

    public record ShotCalculation(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {}
}
