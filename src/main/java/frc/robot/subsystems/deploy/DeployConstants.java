package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class DeployConstants {

    public static final int kMotorID = 6;
    public static final int kRetractedLSChannel = 0;

    public static final boolean kInvertMotor = true;
    public static final boolean kInvertEncoder = false;
    public static final boolean kInvertRetractedLS = true;
    public static final Distance kEncoderOffset = Inches.of(0.0);

    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kAgitationDistance = Inches.of(9.0);
    public static final Distance kMaximumDistance = Inches.of(11.195);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = (5.0 * 4.0 * 24.0 / 22.0);
    public static final double kEncoderToDeployGearing = (22.0 / 24.0);
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Mass kCarriageMass = Kilograms.of(6);
    public static final Distance kDrumRadius = Inches.of(1.5);

    public static final double kPositionConversionFactor =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToDeployGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final MomentOfInertia kDeployMOI = KilogramSquareMeters.of(0.2555);
}
