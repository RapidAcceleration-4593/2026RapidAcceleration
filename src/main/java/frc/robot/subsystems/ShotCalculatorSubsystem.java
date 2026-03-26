package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.util.shooting.ProjectilePhysics.*;
import static frc.robot.util.shooting.ProjectilePhysicsCalibration.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
import frc.robot.util.shooting.ProjectilePhysicsCalibration;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotCalculatorSubsystem extends SubsystemBase {

    public static final int kCalculationIterations = 4;
    public static final double kConvergenceEpsilon = 3e-4;
    public static final double kTwistCompensationFactor = 0.1;
    public static final Time kSystemLatency = Milliseconds.of(80.0);

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
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds robotVelocity = chassisSpeedsSupplier.get();

        // Predicted Pose w/ Latency Compensation.
        Pose2d predictedPose = currentPose.exp(new Twist2d(
                robotVelocity.vxMetersPerSecond * kSystemLatency.in(Seconds),
                robotVelocity.vyMetersPerSecond * kSystemLatency.in(Seconds),
                robotVelocity.omegaRadiansPerSecond * kSystemLatency.in(Seconds)));

        // Calculate Shooter's Total Velocity.
        double shooterVxRobot =
                robotVelocity.vxMetersPerSecond - (robotVelocity.omegaRadiansPerSecond * kPhysicalOffset.getY());
        double shooterVyRobot =
                robotVelocity.vyMetersPerSecond + (robotVelocity.omegaRadiansPerSecond * kPhysicalOffset.getX());
        Translation2d shooterFieldVelocity =
                new Translation2d(shooterVxRobot, shooterVyRobot).rotateBy(predictedPose.getRotation());

        // Shooter Position at Time of Shot.
        Translation2d shooterXY = predictedPose.transformBy(kPhysicalOffset).getTranslation();
        Pose3d realTarget3d = FieldUtil.getTargetPose(predictedPose);
        Translation2d realTargetXY = realTarget3d.toPose2d().getTranslation();
        Distance verticalDistance = realTarget3d.getMeasureZ().minus(kShooterHeight);

        // Iterative Solver for Virtual Target.
        Translation2d virtualTargetXY = realTargetXY;
        Angle hoodAngle = Degrees.zero();
        Time tof = Seconds.of(1.0);

        for (int i = 0; i < kCalculationIterations; i++) {
            Distance virtualDistance = Meters.of(shooterXY.getDistance(virtualTargetXY));

            // Validity Check. Prevents Soft Crash.
            if (virtualDistance.lt(Meters.of(1.25)) || virtualDistance.gt(Meters.of(15.0))) {
                latestResult = ShotResult.invalid();
                return;
            }

            hoodAngle = calculateHood(virtualDistance);
            LinearVelocity requiredLaunchSpeed =
                    ProjectilePhysics.calculateLaunchSpeed(hoodAngle, virtualDistance, verticalDistance);

            // Recalculate Time of Flight.
            tof = ProjectilePhysics.calculateTime(requiredLaunchSpeed, hoodAngle, realTarget3d.getMeasureZ());
            Translation2d nextVirtualTargetXY = realTargetXY.minus(shooterFieldVelocity.times(tof.in(Seconds)));

            // Early Convergence Check.
            if (nextVirtualTargetXY.getDistance(virtualTargetXY) < kConvergenceEpsilon) {
                virtualTargetXY = nextVirtualTargetXY;
                break;
            }
            virtualTargetXY = nextVirtualTargetXY;
        }

        Logger.recordOutput("VirtualTarget", new Pose2d(virtualTargetXY, Rotation2d.kZero));

        // Extract Final Solution.
        Translation2d targetVector = virtualTargetXY.minus(shooterXY);
        Rotation2d angleToTarget = new Rotation2d(targetVector.getX(), targetVector.getY());

        Angle turretAngle = calculateTurret(predictedPose, angleToTarget);
        AngularVelocity shooterVelocity =
                calculateShooter(targetVector, shooterFieldVelocity, verticalDistance, hoodAngle, turretAngle);

        // Final Validity Check.
        if (Double.isNaN(shooterVelocity.in(RadiansPerSecond))) {
            latestResult = ShotResult.invalid();
            return;
        }
        latestResult = new ShotResult(turretAngle, hoodAngle, shooterVelocity, true);

        Logger.recordOutput("ShotCalculation/VirtualTargetPose", new Pose2d(virtualTargetXY, angleToTarget));
        Logger.recordOutput("ShotCalculation/PredictedRobotPose", predictedPose);
        Logger.recordOutput("ShotCalculation/TimeOfFlight", tof.in(Seconds));
    }

    private Angle calculateHood(Distance distance) {
        return Degrees.of(hoodMap.get(distance.in(Meters)));
    }

    private Angle calculateTurret(Pose2d robotPose, Rotation2d angleToTarget) {
        return robotPose.getRotation().getMeasure().minus(angleToTarget.getMeasure());
    }

    private AngularVelocity calculateShooter(
            Translation2d targetVector,
            Translation2d shooterFieldVelocity,
            Distance vDistance,
            Angle hoodAngle,
            Angle turretAngle) {
        Distance hDistance = Meters.of(targetVector.getNorm());
        LinearVelocity requiredLaunchSpeed = ProjectilePhysics.calculateLaunchSpeed(hoodAngle, hDistance, vDistance);

        Translation2d shotDirection = targetVector.div(targetVector.getNorm());
        Translation2d requiredVelocityVector = shotDirection.times(requiredLaunchSpeed.in(MetersPerSecond));

        double effectiveLaunchSpeed =
                requiredVelocityVector.minus(shooterFieldVelocity).getNorm();
        double exitFactor = ProjectilePhysicsCalibration.kDefault.getLinearExitFactor(hDistance, turretAngle);
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

    public boolean isValid() {
        return latestResult.valid();
    }

    public record ShotResult(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {
        public static ShotResult invalid() {
            return new ShotResult(Degrees.zero(), Degrees.zero(), RPM.zero(), false);
        }
    }
}
