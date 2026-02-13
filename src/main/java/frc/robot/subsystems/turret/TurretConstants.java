package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class TurretConstants {

    public static final int kTurretMotorID = 0;

    public static final boolean kInvertTurretEncoder = false;
    public static final Angle kTurretEncoderOffset = Degrees.of(0.0);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Angle kInitialAngle = Degrees.of(0.0);
    public static final Angle kMinimumAngle = Degrees.of(-100.0);
    public static final Angle kMaximumAngle = Degrees.of(100.0);
    public static final Angle kAngleTolerance = Degrees.of(1.0);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 60.0; // 5:4:3
    public static final double kEncoderToTurretGearing = 165.0 / 34.0;
    public static final double kMotorToTurretGearing = kMotorToEncoderGearing * kEncoderToTurretGearing;

    public static final double kPositionConversionFactor = 360.0 / kEncoderToTurretGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final MomentOfInertia kTurretMOI = KilogramSquareMeters.of(0.003);
}
