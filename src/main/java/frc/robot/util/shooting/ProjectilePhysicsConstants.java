package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;

public final class ProjectilePhysicsConstants {

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    public static final Distance kShooterHeight = Inches.of(18.0);
    public static final Distance kWheelRadius = Inches.of(2.0);

    public static final int kCalculationIterations = 4;
    public static final double kConvergenceEpsilon = 3e-4;
    public static final Time kSystemLatency = Milliseconds.of(40.0);

    public static final Distance kFuelRadius = Centimeters.of(7.5);
    public static final AngularVelocity kTowerExitVelocity =
            RadiansPerSecond.of(10.0); // Angular velocity at tower exit.
    public static final double kFrictionLoss = 0.85; // Efficiency of compression.

    // Velocity mps, turret angle rad, exit factor.
    public static final double[][] kExitFactorData = {
        {7.2157, -0.0358, 0.400},
        {7.6178, -0.3159, 0.390},
        {8.0006, 0.70086, 0.395},
        {8.4218, -0.0767, 0.390},
        {7.8484, -1.5037, 0.350},
        {8.0548, 1.21500, 0.390},
		{8.7895, -0.5796, 0.370},
		{8.1888, 2.14742, 0.390},
		{8.0304, -2.3390, 0.360},
		{8.8969, 2.31104, 0.370},
		{8.8849, -0.7583, 0.355},
		{8.9060, 0.65343, 0.360},
		{8.8257, -2.3365, 0.330},
		{8.1637, 1.57358, 0.395},
		{8.0803, -1.4428, 0.360},
		{8.0456, 1.37994, 0.400},
		{8.1221, -1.7379, 0.370},
		{6.4862, 2.80408, 0.370},
		{6.5577, -0.0598, 0.400},
		{6.7053, -0.7293, 0.400},
		{8.1713, 2.93616, 0.375},
		{8.7000, 1.74501, 0.390},
		{8.0536, -2.6555, 0.360},
		{8.2419, 2.85171, 0.375},
		{8.3141, -2.4599, 0.355},
		{7.9750, -1.5851, 0.360},
		{8.2545, -2.5723, 0.355},
		{8.1983, 2.84265, 0.380},
		{7.8949, -2.5504, 0.355},
		{8.5502, 1.62621, 0.390},
		{8.5458, 1.89361, 0.375},
		{7.9235, -1.0960, 0.365},
		{7.0622, -1.1685, 0.385}
    };
}
