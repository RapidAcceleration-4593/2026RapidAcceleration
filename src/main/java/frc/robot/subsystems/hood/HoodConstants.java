package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class HoodConstants {

    public static final int kHoodMotorID = 0;
    public static final int kHoodEncoderChannelA = 0;
    public static final int kHoodEncoderChannelB = 1;
    public static final int kHoodLimitSwitchChannel = 2;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCountsPerRotation = 0.0; // Encoder counts per revolution.
    public static final double kHoodGearRatio = 1.0; // Hood gearing.
    public static final double kDegreesPerPulse = 360.0 / (kCountsPerRotation * kHoodGearRatio);

    public static final Angle kMinimumAngle = Degrees.of(10.0);
    public static final Angle kMaximumAngle = Degrees.of(45.0);
    public static final Angle kAngleTolerance = Degrees.of(0.5);
}
