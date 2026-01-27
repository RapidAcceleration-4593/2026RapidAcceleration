package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public final class ClimberConstants {

    public static final int kClimberMotorID = 0;
    public static final int kClimberEncoderChannelA = 0;
    public static final int kClimberEncoderChannelB = 1;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 1.0;
    public static final double kEncoderToClimberGearing = 1.0;
    public static final double kInchesPerPulse = 1.0;

    public static final Distance kMinimunDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(8.0);
    public static final Distance kDistanceTolerance = Inches.of(0.5);
}
