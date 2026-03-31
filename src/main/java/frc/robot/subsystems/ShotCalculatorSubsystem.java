package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ProjectilePhysicsConstants.*;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.turret.TurretConstants;
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

        // Iterative Solver for Virtual Target.
        Translation2d virtualTargetXY = realTargetXY;
        Angle hoodAngle = calculateHood(Meters.of(virtualTargetXY.getDistance(shooterXY)));

        boolean latestIsValid = true;

        // Extract Final Solution.
        Translation2d targetVector = virtualTargetXY.minus(shooterXY);
        Rotation2d angleToTarget = new Rotation2d(targetVector.getX(), targetVector.getY());

        Angle turretAngle = calculateTurret(currentPose, angleToTarget);
        if (turretAngle.lt(TurretConstants.kMinimumAngle) || turretAngle.gt(TurretConstants.kMaximumAngle)) {
            latestIsValid = false;
        }

        // Final Validity Check.
        AngularVelocity shooterVelocity = calculateShooter(targetVector, verticalDistance, hoodAngle, turretAngle);
        if (Double.isNaN(shooterVelocity.in(RadiansPerSecond))) {
            latestIsValid = false;
        }
        latestResult = new ShotResult(turretAngle, hoodAngle, shooterVelocity, latestIsValid);

        Logger.recordOutput(
                "ShotTuner/RequiredLinearLaunchSpeedMPS", getLaunchSpeed().in(MetersPerSecond));
        Logger.recordOutput("ShotTuner/TurretAngleRad", getTurretAngle().in(Radians));
        Logger.recordOutput("ShotTuner/Valid", isValid());
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(hoodMap.get(distance.in(Meters)));
    }

    private Angle calculateTurret(Pose2d robotPose, Rotation2d angleToTarget) {
        Angle raw = robotPose.getRotation().getMeasure().minus(angleToTarget.getMeasure());
        return Degrees.of(MathUtil.inputModulus(raw.in(Degrees), -180.0, 180.0));
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

    public boolean isInvalid() {
        return !latestResult.valid();
    }

    public record ShotResult(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {
        public static ShotResult invalid() {
            return new ShotResult(Degrees.zero(), Degrees.zero(), RPM.zero(), false);
        }
    }
}
