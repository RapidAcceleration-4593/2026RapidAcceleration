package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    private static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    private static final double[][] kExitFactorData = {
        // { distanceMeters, turretRadians, exitFactor }
        {2.936, 0.026, 0.38},
        {3.586, -0.690, 0.37},
        {2.304, 0.912, 0.40},
        {3.947, -0.607, 0.35},
        {3.769, 1.362, 0.38},
        {5.140, 0.711, 0.35},
        {3.952, 0.387, 0.39},
        {2.481, 0.462, 0.41},
        {1.770, -0.759, 0.38},
        {3.052, -0.807, 0.34},
        {4.729, -0.567, 0.34},
        {3.826, 0.389, 0.36},
        {1.687, -0.980, 0.35},
        {3.475, 0.076, 0.39},
        {2.866, 0.929, 0.39},
        {4.479, -0.390, 0.35},
        {3.361, -0.124, 0.38},
        {3.499, -0.503, 0.36},
        {3.717, 0.619, 0.38},
        {2.045, 0.842, 0.42},
        {3.348, -0.113, 0.36},
        {4.214, -0.260, 0.36},
        {4.340, 0.493, 0.38},
        {3.521, -0.846, 0.36},
        {3.135, -0.949, 0.36},
        {4.372, 0.570, 0.36},
        {3.975, 0.249, 0.36}
    };

    public static double getExitFactor(Distance distance, Angle turretAngle) {
        double d = distance.in(Meters);
        double t = turretAngle.in(Radians);

        double weightedSum = 0;
        double weightedTotal = 0;

        for (double[] p : kExitFactorData) {
            double dd = (d - p[0]) * 2.0;
            double dt = t - p[1];

            double distSquared = dd * dd + dt * dt;

            if (distSquared < 1e-6) {
                return p[2];
            }

            double weight = 1.0 / distSquared;

            weightedSum += weight * p[2];
            weightedTotal += weight;
        }
        double exitFactor = weightedSum / weightedTotal;
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public static LinearVelocity calculateLaunchSpeed(
            Angle hoodAngle, Distance horizontalDistance, Distance verticalDistance) {
        double sin = Math.sin(hoodAngle.in(Radians));
        double cos = Math.cos(hoodAngle.in(Radians));

        double numerator = gravity.in(MetersPerSecondPerSecond) * Math.pow(horizontalDistance.in(Meters), 2);
        double denominator = 2.0 * sin * (horizontalDistance.in(Meters) * cos - verticalDistance.in(Meters) * sin);

        return MetersPerSecond.of(Math.sqrt(numerator / denominator));
    }

    public static Time calculateTime(LinearVelocity launchSpeed, Angle hoodAngle, Distance horizontalDistance) {
        double sin = Math.sin(hoodAngle.in(Radians));

        double numerator = horizontalDistance.in(Meters);
        double denominator = launchSpeed.in(MetersPerSecond) * sin;

        return Seconds.of(numerator / denominator);
    }
}
