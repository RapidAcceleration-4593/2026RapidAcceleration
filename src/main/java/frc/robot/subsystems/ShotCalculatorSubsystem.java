package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ProjectilePhysics.*;
import static frc.robot.util.shooting.ProjectilePhysicsCalibration.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.FieldUtil;
import frc.robot.util.shooting.ProjectilePhysics;
import frc.robot.util.shooting.ProjectilePhysicsCalibration;
import java.util.function.Supplier;

public class ShotCalculatorSubsystem extends SubsystemBase {

    public static final int kCalculationIterations = 4;
    public static final double kConvergenceEpsilon = 3e-4;
    public static final double kTwistCompensationFactor = 0.1;
    public static final Time kSystemLatency = Milliseconds.of(80.0);

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private ShotResult latestResult = ShotResult.invalid();
    private LinearVelocity latestLaunchSpeed = MetersPerSecond.zero();
    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance [meters], Hood [Degrees]
        hoodMap.put(1.25, 12.5);
        hoodMap.put(8.0, 30.0);
    }

    public ShotCalculatorSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    private ShotResult calculateMovingShot(
            Pose2d shooterPose, ChassisSpeeds shooterVelocityFieldRelative, Translation3d targetPos) {
        Translation2d shooterXY = shooterPose.getTranslation();
        Translation2d realTargetXY = targetPos.toTranslation2d();
        Distance verticalDistance = kShooterHeight.minus(targetPos.getMeasureZ());

        Translation2d shooterFieldVelocity = new Translation2d(
                shooterVelocityFieldRelative.vxMetersPerSecond, shooterVelocityFieldRelative.vyMetersPerSecond);

        // Iterative Solver for Virtual Target.
        Translation2d virtualTargetXY = realTargetXY;
        Angle hoodAngle = Degrees.zero();
        Time tof = Seconds.of(1.0);

        boolean latestIsValid = true;
        for (int i = 0; i < kCalculationIterations; i++) {
            Distance virtualDistance = Meters.of(shooterXY.getDistance(virtualTargetXY));

            // Validity Check. Prevents Soft Crash.
            if (virtualDistance.lt(Meters.of(1.25)) || virtualDistance.gt(Meters.of(15.0))) {
                latestIsValid = false;
            }

            hoodAngle = calculateHood(virtualDistance);
            LinearVelocity requiredLaunchSpeed =
                    ProjectilePhysics.calculateLaunchSpeed(hoodAngle, virtualDistance, verticalDistance);

            // Recalculate Time of Flight.
            tof = ProjectilePhysics.calculateTime(requiredLaunchSpeed, hoodAngle, verticalDistance);
            Translation2d nextVirtualTargetXY = realTargetXY.minus(shooterFieldVelocity.times(tof.in(Seconds)));

            // Early Convergence Check.
            if (nextVirtualTargetXY.getDistance(virtualTargetXY) < kConvergenceEpsilon) {
                virtualTargetXY = nextVirtualTargetXY;
                break;
            }
            virtualTargetXY = nextVirtualTargetXY;
        }

        // Extract Final Solution.
        Translation2d targetVector = virtualTargetXY.minus(shooterXY);
        Rotation2d angleToTarget = new Rotation2d(targetVector.getX(), targetVector.getY());

        Angle turretAngle = calculateTurret(shooterPose, angleToTarget);
        if (turretAngle.lt(TurretConstants.kMinimumAngle) || turretAngle.gt(TurretConstants.kMaximumAngle)) {
            latestIsValid = false;
        }

        // Final Validity Check.
        AngularVelocity shooterVelocity =
                calculateShooter(Meters.of(targetVector.getNorm()), verticalDistance, hoodAngle, turretAngle);
        if (Double.isNaN(shooterVelocity.in(RadiansPerSecond))) {
            latestIsValid = false;
            return ShotResult.invalid();
        }
        return new ShotResult(turretAngle, hoodAngle, shooterVelocity, latestIsValid);
    }

    private void calculate() {
        Pose2d currentPose = poseSupplier.get();
        Pose2d shooterPose = currentPose.transformBy(kPhysicalOffset);
        ChassisSpeeds robotVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsSupplier.get(), currentPose.getRotation());
		
        Pose3d realTarget3d = FieldUtil.getTargetPose(currentPose);
        latestResult = calculateMovingShot(shooterPose, robotVelocity, realTarget3d.getTranslation());
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(hoodMap.get(distance.in(Meters)));
    }

    private Angle calculateTurret(Pose2d robotPose, Rotation2d angleToTarget) {
        Angle raw = robotPose.getRotation().getMeasure().minus(angleToTarget.getMeasure());
        return Degrees.of(MathUtil.inputModulus(raw.in(Degrees), -180.0, 180.0));
    }

    private AngularVelocity calculateShooter(
            Distance horizontalDistance, Distance verticalDistance, Angle hoodAngle, Angle turretAngle) {
        LinearVelocity requiredLaunchSpeed =
                ProjectilePhysics.calculateLaunchSpeed(hoodAngle, horizontalDistance, verticalDistance);
        latestLaunchSpeed = requiredLaunchSpeed;

        double exitFactor = ProjectilePhysicsCalibration.kDefault.getLinearExitFactor(requiredLaunchSpeed, turretAngle);
        return RadiansPerSecond.of(requiredLaunchSpeed.in(MetersPerSecond) / (kWheelRadius.in(Meters) * exitFactor));
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
