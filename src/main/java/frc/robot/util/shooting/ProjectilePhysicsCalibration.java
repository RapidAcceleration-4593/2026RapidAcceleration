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

    // Launch speed, turret angle, exit factor
    private static final double[][] kExitFactorData = {};

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
        return offset + speedMult * launchSpeed.in(MetersPerSecond) + angleMult * turretAngle.in(Radians);
    }
}
