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

    // Velocity mps, Turret angle rad, exit factor
    public static final double[][] kExitFactorData = {
        {7.215794239115164, -0.035846669839801434, 0.4},
        {7.617867704715074, -0.3159472474488416, 0.39},
        {8.000617136458493, 0.7008689753404492, 0.395},
        {8.421803672526195, -0.07672113146900184, 0.39},
        {7.84849481133635, -1.5037172589902958, 0.35},
        {8.054884428312278, 1.215, 0.39},
		{8.789550833381195, -0.5796190993297191, 0.37},
		{8.188881447614737, 2.1474266250183898, 0.39},
		{8.03046490141204, -2.3390692554947834, 0.36},
		{8.89694530100527, 2.3110430527926664, 0.37},
		{8.884957102407517, -0.7583466074373744, 0.355},
		{8.90604473058559, 0.6534330406415022, 0.36},
		{8.825768485761639, -2.336542095435308, 0.33},
		{8.16375083958601, 1.573584827856103, 0.395},
		{8.080369311718869, -1.4428135836645057, 0.36},
		{8.04568347353776, 1.3799471551401024, 0.4},
		{8.122143289311829, -1.737967364120772, 0.37},
		{6.486250784123611, 2.8040894704063533, 0.37},
		{6.557782290435724, -0.0598931988636285, 0.4},
		{6.705317508643631, -0.7293756495995432, 0.4},
		{8.120798404618563, 2.6555175002862, 0.395}
    };

    public static final int[][] kTriangles = {
        {43, 40, 39},
        {8, 43, 12},
        {8, 19, 43},
        {33, 40, 43},
        {27, 47, 44},
        {47, 35, 12},
        {35, 8, 12},
        {27, 35, 47},
        {46, 27, 44},
        {35, 49, 9},
        {49, 35, 27},
        {40, 45, 39},
        {45, 42, 39},
        {2, 14, 33},
        {19, 2, 43},
        {2, 33, 43},
        {48, 46, 36},
        {8, 7, 19},
        {7, 2, 19},
        {2, 7, 14},
        {46, 28, 27},
        {28, 48, 38},
        {48, 28, 46},
        {33, 41, 40},
        {41, 37, 40},
        {14, 41, 33},
        {41, 14, 18},
        {37, 4, 40},
        {4, 45, 40},
        {45, 4, 30},
        {30, 4, 18},
        {4, 41, 18},
        {41, 4, 37},
        {34, 5, 42},
        {45, 34, 42},
        {34, 45, 30},
        {34, 30, 18},
        {42, 29, 39},
        {5, 29, 42},
        {5, 15, 10},
        {15, 36, 10},
        {15, 48, 36},
        {32, 17, 38},
        {17, 32, 21},
        {32, 15, 21},
        {15, 32, 48},
        {16, 20, 9},
        {17, 16, 9},
        {7, 0, 14},
        {20, 0, 9},
        {0, 35, 9},
        {35, 0, 8},
        {0, 7, 8},
        {28, 24, 27},
        {49, 24, 9},
        {24, 49, 27},
        {23, 28, 38},
        {23, 24, 28},
        {23, 17, 9},
        {24, 23, 9},
        {22, 15, 5},
        {22, 26, 21},
        {15, 22, 21},
        {31, 5, 10},
        {31, 29, 5},
        {36, 31, 10},
        {29, 31, 39},
        {48, 3, 38},
        {3, 32, 38},
        {32, 3, 48},
        {13, 17, 21},
        {13, 16, 17},
        {26, 13, 21},
        {11, 13, 26},
        {16, 13, 20},
        {13, 11, 18},
        {13, 0, 20},
        {14, 13, 18},
        {0, 13, 14},
        {17, 1, 38},
        {1, 23, 38},
        {23, 1, 17},
        {34, 25, 5},
        {25, 22, 5},
        {25, 34, 18},
        {6, 11, 26},
        {22, 6, 26},
        {11, 6, 18},
        {6, 25, 18},
        {25, 6, 22}
    };
}
