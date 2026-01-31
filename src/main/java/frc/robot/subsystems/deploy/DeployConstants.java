package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class DeployConstants {

    public static final int kDeployMotorID = 6;

    public static final int kInLimitSwitchChannel = 5;
    public static final int kOutLimitSwitchChannel = 6;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int kCountsPerRotation = 8192;
    public static final double kMotorToEncoderGearing = 20.0;
    public static final double kEncoderToDeployGearing = 1.0;
    public static final double kMotorToDeployGearing = kMotorToEncoderGearing * kEncoderToDeployGearing;

    public static final Distance kRetractedDistance = Inches.zero();
    public static final Distance kExtendedDistance = Inches.of(11.2);
    public static final Distance kDistanceTolerance = Inches.of(0.5);

    public static final Distance kDrumDiameter = Inches.of(1.5);
    public static final double kInchesConversionFactor = Math.PI * kDrumDiameter.in(Inches) / kEncoderToDeployGearing;

    public static final Mass kCarriageMass = Pounds.of(13);
}
