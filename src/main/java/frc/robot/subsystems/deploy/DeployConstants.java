package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class DeployConstants {

    public static final int kDeployMotorID = 2;

    public static final int kEncoderChannelA = 0;
    public static final int kEncoderChannelB = 1;

    public static final int kInLimitSwitchChannel = 2;
    public static final int kOutLimitSwitchChannel = 3;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 60.0;
    public static final double kEncoderToDeployGearing = 1.0;
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(11.2);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final Distance kDrumRadius = Inches.of(0.71);
    public static final double kInchesPerPulse =
            2 * Math.PI * kDrumRadius.in(Inches) / kEncoderToDeployGearing / kCountsPerRotation;
    public static final Mass kCarriageMass = Pounds.of(13);
}
