package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public final class DeployConstants {

    public static final int kLeftMotorID = 2;
    public static final int kRightMotorID = 3;

    public static final int kEncoderChannelA = 0;
    public static final int kEncoderChannelB = 1;

    public static final int kLimitSwitchChannel = 2;

    public static final Distance kMinimumDistance = Inches.zero();
    public static final Distance kMaximumDistance = Inches.of(12.0);
}
