package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

public class ProjectilePhysicsCalibration {

    private static final double[][] kExitFactorData = {
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
        {3.975, 0.249, 0.36},
        {2.994, -1.390, 0.33},
        {3.565, -1.149, 0.35},
        {5.391, 0.829, 0.31},
        {4.032, 1.231, 0.375},
        {5.398, 0.660, 0.36},
        {4.099, -0.503, 0.34},
        {2.844, 1.043, 0.376},
        {4.398, 0.908, 0.376},
        {2.367, -0.961, 0.357},
        {4.601, -0.843, 0.352},
        {3.546, 1.391, 0.375},
        {3.942, -0.608, 0.35},
        {5.515, 2.370, 0.36},
        {3.587, 1.972, 0.39},
        {3.303, 1.230, 0.38},
        {4.904, 1.860, 0.37},
        {1.712, 2.441, 0.41},
        {3.366, -1.886, 0.35},
        {4.446, 1.973, 0.38},
        {3.601, -1.752, 0.35},
        {2.268, -1.603, 0.36},
        {4.186, -0.760, 0.358},
        {2.895, -0.944, 0.354}
    };

    public static final ProjectilePhysicsCalibration kDefault = new ProjectilePhysicsCalibration(kExitFactorData);

    private int[][] kTriangles;
    private double offset, distanceMult, angleMult;

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
        System.out.println(params[0]);
        System.out.println(params[1]);
        System.out.println(params[2]);
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

    public double getLinearExitFactor(Distance distance, Angle turretAngle) {
        return offset + distanceMult * distance.in(Meters) + angleMult * turretAngle.in(Radians);
    }
}
