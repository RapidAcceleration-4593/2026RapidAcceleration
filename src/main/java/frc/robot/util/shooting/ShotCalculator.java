package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;

public class ShotCalculator {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public ShotCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public ShotCalculation calculate() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);
        Distance distance = FieldUtil.getDistanceToHub(robotPose);

        Angle hoodAngle = calculateHood(distance);
        AngularVelocity shooterVelocity = calculateShooter(distance);
        Angle turretAngle = calculateTurret(robotPose, distance, shooterVelocity, hoodAngle);

        return new ShotCalculation(turretAngle, hoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        // return Degrees.of(distance.in(Meters) * 5.0);
        return Degrees.of(25.0);
    }

    private AngularVelocity calculateShooter(Distance distance) {
        // return RPM.of(distance.in(Meters) * 750.0);
        return RPM.of(3500.0);
    }

    private Angle calculateTurret(
            Pose2d robotPose, Distance distance, AngularVelocity shooterVelocity, Angle hoodAngle) {
        Pose2d targetPose = FieldUtil.getTargetHubPose();

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
