package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterHeight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;

public class ShotCalculator {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final ProjectilePhysics physics;

    public ShotCalculator(
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, ProjectilePhysics physics) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.physics = physics;
    }

    public ShotSolution calculate() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);
        Distance distance = FieldUtil.getDistanceToHub(robotPose);

        Angle hoodAngle = calculateHood(distance);
        AngularVelocity shooterVelocity = calculateShooter(distance);
        Angle turretAngle = calculateTurret(robotPose, hoodAngle, shooterVelocity);

        return new ShotSolution(turretAngle, hoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(distance.in(Meters) * 5.0);
    }

    private AngularVelocity calculateShooter(Distance distance) {
        return RPM.of(distance.in(Meters) * 750.0);
    }

    private Angle calculateTurret(Pose2d robotPose, Angle hoodAngle, AngularVelocity shooterVelocity) {
        if (FieldUtil.isInAllianceZone(robotPose)) {
            Pose2d targetPose = FieldUtil.getTargetHubPose();
            ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

            LinearVelocity initialVelocity =
                    MetersPerSecond.of(0.5 * shooterVelocity.in(RadiansPerSecond) * Math.cos(hoodAngle.in(Radians)));
            LinearAcceleration gravityConstant = MetersPerSecondPerSecond.of(-9.81);

            Time timeOfFlight = Seconds.of((-initialVelocity.in(MetersPerSecond)
                            - Math.sqrt(Math.pow(initialVelocity.in(MetersPerSecond), 2)
                                    - 2
                                            * gravityConstant.in(MetersPerSecondPerSecond)
                                            * (kShooterHeight.in(Meters) - FieldUtil.kHubHeight.in(Meters))))
                    / gravityConstant.in(MetersPerSecondPerSecond));

            Distance vx = Meters.of(chassisSpeeds.vxMetersPerSecond).times(timeOfFlight.in(Seconds));
            Distance vy = Meters.of(chassisSpeeds.vyMetersPerSecond).times(timeOfFlight.in(Seconds));

            Distance dx =
                    targetPose.getMeasureX().minus(robotPose.getMeasureX()).minus(vx);
            Distance dy =
                    targetPose.getMeasureY().minus(robotPose.getMeasureY()).minus(vy);

            Angle fieldAngle = Radians.of(Math.atan2(dy.in(Meters), dx.in(Meters)));
            return fieldAngle.minus(robotPose.getRotation().getMeasure());
        }

        Angle fieldAngle = FieldUtil.getCurrentAlliance() == Alliance.Blue ? Degrees.of(180) : Degrees.zero();
        return fieldAngle.minus(robotPose.getRotation().getMeasure());
    }
}
