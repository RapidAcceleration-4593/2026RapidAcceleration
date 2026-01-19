package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class TurretConstants {

    public static final int kMotorID = 0;
    public static final int kEncoderChannelA = 0;
    public static final int kEncoderChannelB = 1;

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192.0; // Encoder counts per revolution.
    public static final double kGearRatio = 15.0 / 1.0; // Ratio between ring gear and drive gear.
    public static final double kDegreesPerPulse = 360 / (kCountsPerRotation * kGearRatio);

    public static final Angle kInitialAngle = Degrees.of(0.0);
    public static final Angle kMinimumAngle = Degrees.of(-150.0);
    public static final Angle kMaximumAngle = Degrees.of(150.0);
    public static final Angle kMaximumTolerance = Degrees.of(3.0);

    public static final MomentOfInertia kTurretMOI = KilogramSquareMeters.of(0.0030);
}
