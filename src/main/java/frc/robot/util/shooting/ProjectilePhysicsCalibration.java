package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

public class ProjectilePhysicsCalibration {

    // Launch Speed [m/s], Turret Angle [rad], Exit Factor.
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

    public static final ProjectilePhysicsCalibration kDefault = new ProjectilePhysicsCalibration(kExitFactorData);

    private int[][] kTriangles;
    private double offset, speedMult, angleMult;

    public ProjectilePhysicsCalibration(double[][] exitFactorData) {
        kTriangles = DelaunayTriangulation.triangulate(exitFactorData);
        OLSMultipleLinearRegression ols = new OLSMultipleLinearRegression();
        double[] y = new double[exitFactorData.length];
        double[][] x = new double[exitFactorData.length][2];

        for (int i = 0; i < exitFactorData.length; i++) {
            x[i][0] = exitFactorData[i][0];
            x[i][1] = exitFactorData[i][1];
            y[i] = exitFactorData[i][2];
        }
        ols.newSampleData(y, x);

        double[] params = ols.estimateRegressionParameters();
        this.offset = params[0];
        this.speedMult = params[1];
        this.angleMult = params[2];
    }

    public double getBarycentricExitFactor(Distance distance, Angle turretAngle) {
        double x = distance.in(Meters);
        double y = turretAngle.in(Radians);

        for (int[] tri : kTriangles) {
            double[] p1 = kExitFactorData[tri[0]];
            double[] p2 = kExitFactorData[tri[1]];
            double[] p3 = kExitFactorData[tri[2]];

            double x1 = p1[0], y1 = p1[1], z1 = p1[2];
            double x2 = p2[0], y2 = p2[1], z2 = p2[2];
            double x3 = p3[0], y3 = p3[1], z3 = p3[2];

            double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);

            if (Math.abs(denom) < 1e-9) continue;

            double w1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
            double w2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
            double w3 = 1.0 - w1 - w2;

            if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
                double exitFactor = w1 * z1 + w2 * z2 + w3 * z3;
                return MathUtil.clamp(exitFactor, 0.20, 0.45);
            }
        }
        return kExitFactorData[0][2];
    }

    public double getLinearExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double exitFactor = offset + speedMult * launchSpeed.in(MetersPerSecond) + angleMult * turretAngle.in(Radians);
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public double getQuadraticExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double x = launchSpeed.in(MetersPerSecond);
        double y = turretAngle.in(Radians);
        double exitFactor =
                -0.00924813 * Math.pow(x, 2) - 0.00326423 * Math.pow(y, 2) + 0.132669 * x + 0.00673983 * y - 0.07958;
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public double getCubicExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double x = launchSpeed.in(MetersPerSecond);
        double y = turretAngle.in(Radians);
        double exitFactor = -0.000848441 * Math.pow(x, 3)
                - 0.00120982 * Math.pow(y, 3)
                + 0.0110914 * Math.pow(x, 2)
                - 0.00285624 * Math.pow(y, 2)
                - 0.0312852 * x
                + 0.0134458 * y
                + 0.364219;
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public double getInterrelationalExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double x = launchSpeed.in(MetersPerSecond);
        double y = turretAngle.in(Radians);
        double exitFactor = -0.00761085 * Math.pow(x, 3)
                - 0.00107918 * Math.pow(y, 3)
                + 0.171929 * Math.pow(x, 2)
                - 0.00267933 * Math.pow(y, 2)
                + 0.00417471 * x * y
                - 1.29963 * x
                - 0.0215811 * y
                + 3.67785;
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public double getHarmonicExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double x = launchSpeed.in(MetersPerSecond);
        double y = turretAngle.in(Radians);

        double cosA = Math.cos(y);
        double sinA = Math.sin(y);
        double cos2A = Math.cos(2 * y);
        double sin2A = Math.sin(2 * y);

        double exitFactor = -0.145568
                + (0.147044 * x)
                + (-0.010168 * x * x)
                + (0.049946 * cosA)
                + (-0.006980 * sinA)
                + (-0.004910 * x * cosA)
                + (0.002995 * x * sinA)
                + (0.002020 * cos2A)
                + (-0.005352 * sin2A);
        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }
}
