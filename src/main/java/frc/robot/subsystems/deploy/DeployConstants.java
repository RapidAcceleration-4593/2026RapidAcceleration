package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class DeployConstants {

    public static final int kDeployMotorID = 6;

    public static final int kRetractedLSChannel = 5;
    public static final int kExtendedLSChannel = 6;

    public static final boolean kInvertInLS = false;
    public static final boolean kInvertOutLS = false;
    public static final boolean kInvertDeployEncoder = false;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final AngularVelocity kCruiseVelocity = RPM.of(100000);
    public static final AngularAcceleration kMaxAcceleration = RPM.per(Second).of(2000000);
    public static final Distance kRetractedDistance = Inches.zero();
    public static final Distance kExtendedDistance = Inches.of(8.0);
    public static final Distance kDistanceTolerance = Inches.of(0.1);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 20.0;
    public static final double kEncoderToDeployGearing = 1.0;
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Distance kDrumRadius = Inches.of(0.75);
    public static final double kInchesConversionFactor = 2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToDeployGearing;
    public static final Mass kCarriageMass = Pounds.of(13);
}
