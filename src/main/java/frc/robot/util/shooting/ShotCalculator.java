package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ShotCalculatorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;

public class ShotCalculator {

    private final ProjectilePhysics physics;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public ShotCalculator(
            ProjectilePhysics physics, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.physics = physics;
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public ShotCalculation calculate() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);
        Pose2d targetPose = FieldUtil.getTargetHubPose().toPose2d();

        Distance horizontalDistance = FieldUtil.getDistanceToHub(robotPose);
        Distance verticalDistance = FieldUtil.getTargetHubPose().getMeasureZ().minus(kShooterHeight);

        Angle hoodAngle = calculateHood(horizontalDistance);
        AngularVelocity shooterVelocity = calculateShooter(hoodAngle, horizontalDistance, verticalDistance);
        Angle turretAngle = calculateTurret(robotPose, targetPose);

        return new ShotCalculation(turretAngle, hoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(distance.in(Meters) * 3.57 + 7.145);
    }

    private AngularVelocity calculateShooter(Angle hoodAngle, Distance horizontalDistance, Distance verticalDistance) {
        LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

        double sin = Math.sin(hoodAngle.in(Radians));
        double cos = Math.cos(hoodAngle.in(Radians));

        double numerator = gravity.in(MetersPerSecondPerSecond) * Math.pow(horizontalDistance.in(Meters), 2);
        double denominator = 2.0 * sin * (horizontalDistance.in(Meters) * cos - verticalDistance.in(Meters) * sin);
        LinearVelocity velocity = MetersPerSecond.of(Math.sqrt(numerator / denominator));

        return RadiansPerSecond.of(velocity.in(MetersPerSecond) / (kWheelRadius.in(Meters) * kExitVelocityFactor));
    }

    private Angle calculateTurret(Pose2d robotPose, Pose2d targetPose) {
        // if (!FieldUtil.isInAllianceZone(robotPose)) {
        //     Angle fieldAngle = FieldUtil.getCurrentAlliance() == Alliance.Blue ? Degrees.of(180) : Degrees.zero();
        //     return robotPose.getRotation().getMeasure().minus(fieldAngle);
        // }

        // ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
        // LinearVelocity initialVelocity =
        //         MetersPerSecond.of(0.5 * shooterVelocity.in(RadiansPerSecond) * Math.cos(hoodAngle.in(Radians)));
        // Time timeOfFlight = Seconds.of(
        //         distance.in(Meters) / (initialVelocity.in(MetersPerSecond) * Math.sin(hoodAngle.in(Radians))));

        // Distance vx = Meters.of(chassisSpeeds.vxMetersPerSecond).times(1.25);
        // Distance vy = Meters.of(chassisSpeeds.vyMetersPerSecond).times(1.25);

        Distance dx = targetPose.getMeasureX().minus(robotPose.getMeasureX()); // .plus(vx);
        Distance dy = targetPose.getMeasureY().minus(robotPose.getMeasureY()); // .plus(vy);

        Angle fieldAngle = Radians.of(Math.atan2(dy.in(Meters), dx.in(Meters)));
        return robotPose.getRotation().getMeasure().minus(fieldAngle);
    }

    public record ShotCalculation(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {}
}
