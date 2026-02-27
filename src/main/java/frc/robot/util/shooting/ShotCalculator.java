package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ShotCalculatorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.FieldUtil;
import java.util.List;
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

        Translation3d start = new Translation3d(robotPose.getMeasureX(), robotPose.getMeasureY(), kShooterHeight);
        Translation3d target3D = ShotCalculatorConstants.kTarget;

        Distance horizontalDistance = FieldUtil.getDistanceToHub(robotPose);

        Angle hoodAngle = calculateHood(horizontalDistance);

        AngularVelocity shooterVelocity =
                calculateShooter(start, hoodAngle, horizontalDistance, target3D.getMeasureZ());
        Angle turretAngle = calculateTurret(robotPose, targetPose);

        return new ShotCalculation(turretAngle, hoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        // return Degrees.of(distance.in(Meters) * 5.0);
        return Degrees.of(25.0);
    }

    private AngularVelocity calculateShooter(
            Translation3d start, Angle hoodAngle, Distance targetDistance, Distance targetHeight) {
        // double low = 1.0;
        // double high = 30.0;

        // for (int i = 0; i < 30; i++) {
        //     double mid = (low + high) / 2.0;
        //     double error = simulateHeightError(start, hoodAngle, mid, targetDistance, targetHeight);

        //     if (error > 0) {
        //         high = mid;
        //     } else {
        //         low = mid;
        //     }
        // }

        // LinearVelocity velocity = MetersPerSecond.of((low + high) / 2.0);
        // return RadiansPerSecond.of(velocity.in(MetersPerSecond) / (kExitVelocityFactor * kWheelRadius.in(Meters)));
        return RPM.of(3500.0);
    }

    private double simulateHeightError(
            Translation3d start, Angle hoodAngle, double velocity, Distance targetDistance, Distance targetHeight) {
        double vx = velocity * Math.cos(hoodAngle.in(Radians));
        double vy = velocity * Math.sin(hoodAngle.in(Radians));

        double[] state = new double[] {0.0, vx, start.getMeasureZ().in(Meters), vy};

        double dt = 0.01;
        while (state[0] < targetDistance.in(Meters) && state[2] >= 0) {
            state = physics.integrateState(state, dt);
        }

        return state[2] - targetHeight.in(Meters);
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

    /**
     * Calculates the distance between the {@code target} point and the closest point lying on the {@code trajectory}.
     * Linearly interpolates between the {@link Translation2d}s describing the trajectory.
     *
     * @param trajectory The trajectory to be used in the calculation.
     * @param target The target to be used in the calculation.
     * @return The closest distance between the {@code trajectory} and the {@code target}.
     */
    private Distance calculateTrajectoryError(List<Translation2d> trajectory, Translation2d target) {
        return null;
    }

    public record ShotCalculation(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {}
}
