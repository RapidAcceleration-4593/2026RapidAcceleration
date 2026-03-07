package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ProjectilePhysicsConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import frc.robot.util.shooting.ProjectilePhysics;
import java.util.function.Supplier;

public class ShotCalculatorSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private ShotResult latestResult = ShotResult.invalid();

    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance [meters], Hood [Degrees]
        hoodMap.put(1.5, 12.5);
        hoodMap.put(5.0, 25.0);
    }

    public ShotCalculatorSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    private void calculate() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);
        Pose3d targetPose3d = FieldUtil.getTargetPose(robotPose);
        Pose2d targetPose2d = targetPose3d.toPose2d();

        Distance realDistance = Meters.of(robotPose.getTranslation().getDistance(targetPose2d.getTranslation()));
        if (realDistance.lt(Meters.of(0.5)) || realDistance.gt(Meters.of(10.0))) {
            latestResult = ShotResult.invalid();
            return;
        }

        ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsSupplier.get(), robotPose.getRotation());

        Pose2d virtualTarget = targetPose2d;
        Distance verticalDistance = targetPose3d.getMeasureZ().minus(kShooterHeight);
        Time tof = Seconds.zero();

        for (int i = 0; i < 3; i++) {
            Distance virtualDistance =
                    Meters.of(virtualTarget.getTranslation().getDistance(robotPose.getTranslation()));
            Angle hoodAngle = calculateHood(virtualDistance);

            LinearVelocity launchSpeed =
                    ProjectilePhysics.calculateLaunchSpeed(hoodAngle, virtualDistance, verticalDistance);
            tof = ProjectilePhysics.calculateTime(launchSpeed, hoodAngle, virtualDistance);

            virtualTarget = new Pose2d(
                    virtualTarget.getMeasureX().minus(Meters.of(chassisSpeeds.vxMetersPerSecond * tof.in(Seconds))),
                    virtualTarget.getMeasureY().minus(Meters.of(chassisSpeeds.vyMetersPerSecond * tof.in(Seconds))),
                    virtualTarget
                            .getRotation()
                            .minus(new Rotation2d(Radians.of(chassisSpeeds.omegaRadiansPerSecond * tof.in(Seconds)))));
        }

        Distance finalVirtualDistance =
                Meters.of(virtualTarget.getTranslation().getDistance(robotPose.getTranslation()));
        Angle finalHoodAngle = calculateHood(finalVirtualDistance);
        LinearVelocity finalLaunchSpeed =
                ProjectilePhysics.calculateLaunchSpeed(finalHoodAngle, finalVirtualDistance, verticalDistance);

        Angle turretAngle = calculateTurret(robotPose, virtualTarget);
        AngularVelocity shooterVelocity = calculateShooter(finalLaunchSpeed, finalVirtualDistance, turretAngle);

        latestResult = new ShotResult(turretAngle, finalHoodAngle, shooterVelocity, true);
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(hoodMap.get(distance.in(Meters)));
    }

    private AngularVelocity calculateShooter(LinearVelocity launchSpeed, Distance distance, Angle turretAngle) {
        // double kExitVelocityFactor = SmartDashboard.getNumber("ExitVelocityFactor", 0.35);
        double kExitVelocityFactor = ProjectilePhysics.getExitFactor(distance, turretAngle);
        return RadiansPerSecond.of(launchSpeed.in(MetersPerSecond) / (kWheelRadius.in(Meters) * kExitVelocityFactor));
    }

    private Angle calculateTurret(Pose2d robotPose, Pose2d virtualTarget) {
        Distance dx = virtualTarget.getMeasureX().minus(robotPose.getMeasureX());
        Distance dy = virtualTarget.getMeasureY().minus(robotPose.getMeasureY());

        Angle fieldAngle = Radians.of(Math.atan2(dy.in(Meters), dx.in(Meters)));
        return robotPose.getRotation().getMeasure().minus(fieldAngle);
    }

    public Angle getHoodAngle() {
        return latestResult.hoodAngle();
    }

    public AngularVelocity getShooterVelocity() {
        return latestResult.shooterVelocity();
    }

    public Angle getTurretAngle() {
        return latestResult.turretAngle();
    }

    public boolean isValid() {
        return latestResult.valid();
    }

    @Override
    public void periodic() {
        calculate();
    }

    public record ShotResult(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {
        public static ShotResult invalid() {
            return new ShotResult(Degrees.zero(), Degrees.zero(), RPM.zero(), false);
        }
    }
}
