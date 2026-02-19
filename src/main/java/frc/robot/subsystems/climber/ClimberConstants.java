package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ClimberConstants {

    public static final int kMotorID = 2;

    public static final boolean kInvertMotor = false;
    public static final boolean kInvertEncoder = false;
    public static final Distance kEncoderOffset = Inches.of(0.0);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(4.0);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = (9.0 * 10.0 * 3.0);
    public static final double kEncoderToClimberGearing = 1.0;
    public static final double kMotorToClimberGearing = kMotorToEncoderGearing * kEncoderToClimberGearing;

    public static final Mass kCarriageMass = Kilograms.of(2);
    public static final Distance kDrumRadius = Inches.of(0.5);

    public static final double kPositionConversionFactor =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToClimberGearing;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final MomentOfInertia kClimberMOI = KilogramSquareMeters.of(0.0);
}
