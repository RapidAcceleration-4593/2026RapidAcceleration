package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class DeployConstants {

    public static final int kMotorID = 6;

    public static final int kRetractedLSChannel = 1;
    public static final int kExtendedLSChannel = 2;

    public static final boolean kInvertMotor = true;
    public static final boolean kInvertEncoder = false;
    public static final boolean kInvertInLS = true;
    public static final boolean kInvertOutLS = true;

    /** Does applying a positive voltage to the motor extend the deploy mechanism? */
    public static final boolean kPositiveVoltageExtends = true;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kAgitationDistance = Inches.of(6.0);
    public static final Distance kMaximumDistance = Inches.of(11.195);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = (5.0 * 4.0);
    public static final double kEncoderToDeployGearing = (24.0 / 24.0);
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Mass kCarriageMass = Pounds.of(13);
    public static final Distance kDrumRadius = Inches.of(0.75);

    public static final double kPositionConversionFactor =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToDeployGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final MomentOfInertia kDeployMOI = KilogramSquareMeters.of(0.2555);
}
