package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class HoodConstants {

    public static final int kHoodMotorID = 2;

    public static final int kHoodLimitSwitchChannel = 2;

    public static final boolean kInvertHoodLS = false;
    public static final boolean kInvertHoodEncoder = false;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final AngularVelocity kCruiseVelocity = RPM.zero();
    public static final AngularAcceleration kMaxAcceleration = RPM.per(Second).zero();
    public static final Angle kMinimumAngle = Degrees.of(20.0);
    public static final Angle kMaximumAngle = Degrees.of(45.0);
    public static final Angle kAngleTolerance = Degrees.of(1.0);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 125.0;
    public static final double kEncoderToHoodGearing = 6.3125;
    public static final double kMotorToHoodGearing = kMotorToEncoderGearing * kEncoderToHoodGearing;
    public static final double kDegreesConversionFactor = 360.0 / kEncoderToHoodGearing;

    public static final Transform2d kPhysicalOffset =
            new Transform2d(new Translation2d(Inches.of(-5.375), Inches.zero()), new Rotation2d());
    public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.04);
}
