package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.ExtraUnits.PoundSquareInches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class HoodConstants {

    public static final int kHoodMotorID = 4;

    public static final int kHoodEncoderChannelA = 0;
    public static final int kHoodEncoderChannelB = 1;

    public static final int kHoodLimitSwitchChannel = 2;
    public static final boolean kHoodLimitSwitchInverted = false;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192; // Unscaled encoder counts per revolution.
    public static final double kMotorToEncoderGearing = 60.0; // Gearing between drive motor and hood axle.
    public static final double kEncoderToHoodGearing = 5.75;
    public static final double kMotorToHoodGearing = kMotorToEncoderGearing * kEncoderToHoodGearing;
    public static final double kDegreesPerPulse = 360.0 / (kCountsPerRotation * kEncoderToHoodGearing);

    public static final MomentOfInertia kHoodMOI = PoundSquareInches.of(200); // Pure guestimation.

    public static final Angle kMinimumAngle = Degrees.of(10.0);
    public static final Angle kMaximumAngle = Degrees.of(45.0);
    public static final Angle kAngleTolerance = Degrees.of(0.5);
}
