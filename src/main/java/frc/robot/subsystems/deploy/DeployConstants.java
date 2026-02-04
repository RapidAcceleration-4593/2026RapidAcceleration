package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public final class DeployConstants {

    public static final int kDeployMotorID = 6;

    public static final int kRetractedLSChannel = 1;
    public static final int kExtendedLSChannel = 2;

    public static final boolean kInvertInLS = false;
    public static final boolean kInvertOutLS = false;
    public static final boolean kInvertDeployEncoder = false;

    public static final double kP = 0.0; // 0.3
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(11.195);
    public static final Distance kDistanceTolerance = Inches.of(0.5);
    public static final LinearVelocity kCruiseVelocity = InchesPerSecond.of(8);
    public static final LinearAcceleration kMaxAcceleration = InchesPerSecondPerSecond.of(12);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 20.0;
    public static final double kEncoderToDeployGearing = 1.0;
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Mass kCarriageMass = Pounds.of(13);
    public static final Distance kDrumRadius = Inches.of(0.75);

    public static final double kPositionConversionFactor =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToDeployGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
}
