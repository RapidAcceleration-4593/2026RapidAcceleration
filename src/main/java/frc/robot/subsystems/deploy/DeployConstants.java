package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class DeployConstants {

    public static final int kLeftMotorID = 2;
    public static final int kRightMotorID = 3;

    public static final int kEncoderChannelA = 0;
    public static final int kEncoderChannelB = 1;

    public static final int kLimitSwitchChannel = 2;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(12.0);

    public static final double kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 1.0;
    public static final double kEncoderToDeployGearing = 1.0;
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;
    public static final double kDistancePerPulse = 1.0;

    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.001); // Assumed.
}
