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
import org.littletonrobotics.junction.Logger;

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
        hoodMap.put(6.0, 30.0);
    }

    public ShotCalculatorSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    private void calculate() {
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds robotVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsSupplier.get(), currentPose.getRotation());

        // Calculate Shooter's Total Velocity.
        Translation2d shooterFieldVelocity =
                new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        // Shooter Position at Time of Shot.
        Translation2d shooterXY = currentPose.transformBy(kPhysicalOffset).getTranslation();
        Pose3d realTarget3d = FieldUtil.getTargetPose(currentPose);
        Translation2d realTargetXY = realTarget3d.toPose2d().getTranslation();
        Distance verticalDistance = realTarget3d.getMeasureZ().minus(kShooterHeight);

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

        Angle turretAngle = calculateTurret(currentPose, angleToTarget);
        if (turretAngle.lt(TurretConstants.kMinimumAngle) || turretAngle.gt(TurretConstants.kMaximumAngle)) {
            latestIsValid = false;
        }

        // Final Validity Check.
        AngularVelocity shooterVelocity =
                calculateShooter(Meters.of(targetVector.getNorm()), verticalDistance, hoodAngle, turretAngle);
        if (Double.isNaN(shooterVelocity.in(RadiansPerSecond))) {
            latestIsValid = false;
        }
        latestResult = new ShotResult(turretAngle, hoodAngle, shooterVelocity, latestIsValid);

        Logger.recordOutput("ShotCalculation/VirtualTargetPose", new Pose2d(virtualTargetXY, angleToTarget));
        Logger.recordOutput("ShotCalculation/ChassisSpeeds", robotVelocity);
        Logger.recordOutput("ShotCalculation/PredictedRobotPose", currentPose);
        Logger.recordOutput("ShotCalculation/TimeOfFlight", tof.in(Seconds));
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

        double exitFactor = ProjectilePhysicsCalibration.kDefault.getLinearExitFactor(horizontalDistance, turretAngle);
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
