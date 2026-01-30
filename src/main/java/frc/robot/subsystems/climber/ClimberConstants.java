package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ClimberConstants {

    public static final int kClimberMotorID = 0;
    public static final int kClimberEncoderChannelA = 0;
    public static final int kClimberEncoderChannelB = 1;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 10;
    public static final double kEncoderToClimberGearing = 1;
    public static final double kMotorToClimberGearing = kMotorToEncoderGearing * kEncoderToClimberGearing;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(8.0);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final Distance kDrumRadius = Inches.of(0.5);
    public static final double kInchesPerPulse =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToClimberGearing / kCountsPerRotation;
    public static final Mass kCarriageMass = Kilograms.of(2);
}
