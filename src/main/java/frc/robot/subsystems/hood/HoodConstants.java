package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class HoodConstants {

    public static final int kHoodMotorID = 0;
    public static final int kHoodEncoderChannelA = 0;
    public static final int kHoodEncoderChannelB = 0;
    public static final int kHoodLimitSwitchChannel = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCountsPerRotation = 0.0; // Encoder counts per revolution.
    public static final double kHoodGearRatio = 0.0; // Hood gearing.
    public static final double kDegreesPerPulse = 360.0 / (kCountsPerRotation * kHoodGearRatio);

    public static final Angle kInitialAngle = Degrees.of(0.0);
    public static final Angle kMaximumAngle = Degrees.of(0.0);
    public static final Angle kMinimumAngle = Degrees.of(15.0);
    public static final Angle kToleranceAngle = Degrees.of(0.5);

    public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.0005);
    public static final Distance kHoodLength = Inches.of(3.0);
}
