package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class HoodConstants {

    public static final int kMotorID = 8;
    public static final int kLSChannel = 1;

    public static final boolean kInvertMotor = true;
    public static final boolean kInvertEncoder = true;
    public static final boolean kInvertLS = true;

    public static final double kP = 0.3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.135;

    public static final Angle kMinimumAngle = Degrees.of(12.5);
    public static final Angle kMaximumAngle = Degrees.of(30);
    public static final Angle kAngleTolerance = Degrees.of(0.1);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = (5.0 * 4.0 * 3.0) * (32.0 / 34.0);
    public static final double kEncoderToHoodGearing = (296.0 / 18.0);
    public static final double kMotorToHoodGearing = kMotorToEncoderGearing * kEncoderToHoodGearing;

    public static final double kPositionConversionFactor = 360.0 / kEncoderToHoodGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.04);
}
