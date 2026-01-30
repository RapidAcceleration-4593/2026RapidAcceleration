package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class HoodConstants {

    public static final int kHoodMotorID = 4;

    public static final int kHoodEncoderChannelA = 6;
    public static final int kHoodEncoderChannelB = 7;

    public static final int kHoodLimitSwitchChannel = 8;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 125.0;
    public static final double kEncoderToHoodGearing = 6.3125;
    public static final double kMotorToHoodGearing = kMotorToEncoderGearing * kEncoderToHoodGearing;
    public static final double kDegreesPerPulse = 360.0 / (kCountsPerRotation * kEncoderToHoodGearing);

    public static final Angle kMinimumAngle = Degrees.of(10.0);
    public static final Angle kMaximumAngle = Degrees.of(45.0);
    public static final Angle kAngleTolerance = Degrees.of(3.0);

    public static final Transform2d kPhysicalOffset =
            new Transform2d(new Translation2d(Inches.of(-5.375), Inches.zero()), new Rotation2d());
    public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.04);
}
