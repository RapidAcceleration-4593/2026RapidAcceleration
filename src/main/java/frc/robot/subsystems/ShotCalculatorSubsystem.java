package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ProjectilePhysicsConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import frc.robot.util.shooting.ProjectilePhysics;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShotCalculatorSubsystem extends SubsystemBase {

    private final LoggedNetworkNumber networkExitFactor;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private ShotResult latestResult = ShotResult.invalid();
    private LinearVelocity latestLaunchSpeed = MetersPerSecond.zero();
    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance [meters], Hood [Degrees]
        hoodMap.put(1.25, 12.5);
        hoodMap.put(6.0, 30.0);
    }

    public ShotCalculatorSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.networkExitFactor = new LoggedNetworkNumber("Tuning/ExitFactor", 0.3);
    }

    private void calculate() {
        Pose2d currentPose = poseSupplier.get();
        Translation2d shooterXY = currentPose.transformBy(kPhysicalOffset).getTranslation();

        Pose3d realTarget3d = FieldUtil.getTargetHubPose();
        Translation2d realTargetXY = realTarget3d.toPose2d().getTranslation();
        Distance verticalDistance = realTarget3d.getMeasureZ().minus(kShooterHeight);
        Translation2d targetVector = realTargetXY.minus(shooterXY);

        Angle hoodAngle = calculateHood(Meters.of(targetVector.getDistance(Translation2d.kZero)));
        Rotation2d angleToTarget = new Rotation2d(targetVector.getX(), targetVector.getY());

        Angle turretAngle = calculateTurret(currentPose, angleToTarget);
        AngularVelocity shooterVelocity = calculateShooter(targetVector, verticalDistance, hoodAngle, turretAngle);

        // Final Validity Check.
        if (Double.isNaN(shooterVelocity.in(RadiansPerSecond))) {
            latestResult = ShotResult.invalid();
            return;
        }
        latestResult = new ShotResult(turretAngle, hoodAngle, shooterVelocity, true);

        Logger.recordOutput("ShotTuner/RequiredLinearLaunchSpeed", getLaunchSpeed());
        Logger.recordOutput("ShotTuner/TurretAngle", getTurretAngle());
        Logger.recordOutput("ShotTuner/Valid", isValid());
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(hoodMap.get(distance.in(Meters)));
    }

    private Angle calculateTurret(Pose2d robotPose, Rotation2d angleToTarget) {
        return robotPose.getRotation().getMeasure().minus(angleToTarget.getMeasure());
    }

    private AngularVelocity calculateShooter(
            Translation2d targetVector, Distance vDistance, Angle hoodAngle, Angle turretAngle) {
        Distance hDistance = Meters.of(targetVector.getNorm());
        LinearVelocity requiredLaunchSpeed = ProjectilePhysics.calculateLaunchSpeed(hoodAngle, hDistance, vDistance);
        latestLaunchSpeed = requiredLaunchSpeed;

        Translation2d shotDirection = targetVector.div(targetVector.getNorm());
        Translation2d requiredVelocityVector = shotDirection.times(requiredLaunchSpeed.in(MetersPerSecond));

        double effectiveLaunchSpeed = requiredVelocityVector.getNorm();
        double exitFactor = networkExitFactor.get();
        Logger.recordOutput("ShotTuner/ExitFactor", exitFactor);
        return RadiansPerSecond.of(effectiveLaunchSpeed / (kWheelRadius.in(Meters) * exitFactor));
    }

    @Override
    public void periodic() {
        calculate();
    }

    public Angle getHoodAngle() {
        return latestResult.hoodAngle();
    }

    public Angle getTurretAngle() {
        return latestResult.turretAngle();
    }

    public AngularVelocity getShooterVelocity() {
        return latestResult.shooterVelocity();
    }

    public LinearVelocity getLaunchSpeed() {
        return latestLaunchSpeed;
    }

    public boolean isValid() {
        return latestResult.valid();
    }

    public record ShotResult(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {
        public static ShotResult invalid() {
            return new ShotResult(Degrees.zero(), Degrees.zero(), RPM.zero(), false);
        }
    }
}
