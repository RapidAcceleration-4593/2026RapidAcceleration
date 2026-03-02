package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;

public final class ProjectilePhysicsConstants {

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    public static final Distance kShooterHeight = Inches.of(18.0);
    public static final Distance kWheelRadius = Inches.of(2.0);

    public static final double[][] kExitFactorData = {
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

    public static final int[][] kTriangles = {
        {4, 25, 5},
        {8, 19, 12},
        {9, 8, 12},
        {24, 9, 12},
        {9, 24, 23},
        {24, 10, 23},
        {0, 8, 9},
        {18, 25, 4},
        {14, 18, 4},
        {13, 21, 26},
        {0, 13, 14},
        {13, 18, 14},
        {7, 0, 14},
        {8, 7, 19},
        {0, 7, 8},
        {18, 6, 25},
        {23, 3, 1},
        {10, 3, 23},
        {25, 22, 5},
        {21, 22, 26},
        {22, 6, 26},
        {6, 22, 25},
        {17, 16, 9},
        {17, 23, 1},
        {17, 9, 23},
        {3, 17, 1},
        {16, 17, 13},
        {13, 17, 21},
        {17, 3, 21},
        {20, 0, 9},
        {16, 20, 9},
        {20, 13, 0},
        {20, 16, 13},
        {7, 2, 19},
        {2, 7, 14},
        {19, 2, 4},
        {2, 14, 4},
        {13, 11, 18},
        {11, 6, 18},
        {11, 13, 26},
        {6, 11, 26},
        {15, 3, 10},
        {3, 15, 21},
        {15, 22, 21},
        {15, 10, 5},
        {22, 15, 5}
    };
}
