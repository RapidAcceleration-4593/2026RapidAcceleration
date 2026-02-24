package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;
import org.littletonrobotics.junction.Logger;

/**
 * A class wrapping the calculations concerning the dynamics of projectiles, including drag. For more information on the
 * mathematics see https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/.
 */
public class ProjectilePhysics {
    private double projectileMassKg;
    private double crossSectAreaSqMeter;
    private double dragCoefficient;

    private double terminalVelocity; // Meters per second

    private static double gravity = 9.8; // Meters per second squared
    private static double airDensity = 1.225; // Kg per cubic meter

    public static final ProjectilePhysics kFuelPhysics = new ProjectilePhysics(0.2257, 0.01767, 0.47);

    public ProjectilePhysics(double projectileMassKg, double crossSectAreaSqMeter, double dragCoefficient) {
        this.projectileMassKg = projectileMassKg;
        this.crossSectAreaSqMeter = crossSectAreaSqMeter;
        this.dragCoefficient = dragCoefficient;

        terminalVelocity =
                Math.sqrt((2 * projectileMassKg * gravity) / (dragCoefficient * airDensity * crossSectAreaSqMeter));
    }

    /**
     * Calculates the y velocity of this projectile, assuming it was launched {@code time} seconds ago with a velocity
     * {@code initialVelocity} in the y axis.
     *
     * @param initialVelocity The initial y velocity of the projectile, in meters per second.
     * @param time How long ago this projectile was launched, in seconds.
     * @return The velocity of the projectile along the y axis, in meters per second.
     */
    private double calculateVY(double initialVelocity, double time) {
        return terminalVelocity
                * (initialVelocity - terminalVelocity * Math.tan(gravity * time / terminalVelocity))
                / (terminalVelocity + initialVelocity * Math.tan(gravity * time / terminalVelocity));
    }

    /**
     * Calculates the y position of the projectile, assuming it was launched {@code time} seconds ago with a velocity of
     * {@code initialVelocity} in the y axis at a height of {@code initialHeight}.
     *
     * @param initialHeight The initial height of the projectile, in meters.
     * @param initialVelocity The initial y velocity of the projectile, in meters per second.
     * @param time How long ago this projectile was launched, in seconds.
     * @return The height of the projectile, in meters.
     */
    private double calculateY(double initialHeight, double initialVelocity, double time) {
        double vY = calculateVY(initialVelocity, time);
        return Math.pow(terminalVelocity, 2)
                        / (2 * gravity)
                        * Math.log((Math.pow(initialVelocity, 2) + Math.pow(terminalVelocity, 2))
                                / (Math.pow(vY, 2) + Math.pow(terminalVelocity, 2)))
                + initialHeight;
    }

    /**
     * Calculates the x velocity of this projectile, assuming it was launched {@code time} seconds ago with a velocity
     * {@code initialVelocity} in the x axis.
     *
     * @param initialVelocity The initial x velocity of the projectile, in meters per second.
     * @param time How long ago this projectile was launched, in seconds.
     * @return The velocity of the projectile, along the x axis, in meters per second.
     */
    private double calculateVX(double initialVelocity, double time) {
        return (Math.pow(terminalVelocity, 2) * initialVelocity)
                / (Math.pow(terminalVelocity, 2) + (gravity * initialVelocity * time));
    }

    /**
     * Calculates the x position of the projectile, assuming it was launched {@code time} seconds ago with a velocity of
     * {@code initialVelocity} in the x axis at a position of {@code initialPosition}.
     *
     * @param initialHeight The initial position of the projectile, in meters.
     * @param initialVelocity The initial x velocity of the projectile, in meters per second.
     * @param time How long ago this projectile was launched, in seconds.
     * @return The position of the projectile, in meters.
     */
    private double calculateX(double initialPosition, double initialVelocity, double time) {
        return (Math.pow(terminalVelocity, 2) / gravity)
                        * Math.log((Math.pow(terminalVelocity, 2) + (gravity * initialVelocity * time))
                                / Math.pow(terminalVelocity, 2))
                + initialPosition;
    }

    public List<Translation2d> calculateTrajectory(
            Translation2d initialPosition, double angle, double initialVelocity, double duration, double timestep) {
        List<Translation2d> points = new ArrayList<>();
        double time = 0;
        double initialVelocityX = initialVelocity * Math.cos(angle);
        double initialVelocityY = initialVelocity * Math.sin(angle);

        while (time < duration) {
            double x = calculateX(initialPosition.getX(), initialVelocityX, time);
            double y = calculateY(initialPosition.getY(), initialVelocityY, time);
            points.add(new Translation2d(x, y));
            time += timestep;
        }
        return points;
    }

    private Translation2d[] foo(int size) {
        return new Translation2d[size];
    }

    public void runSimulation() {
        var points = calculateTrajectory(Translation2d.kZero, Units.degreesToRadians(45), 100, 10, 0.1);
        IntFunction<Translation2d[]> generator = (size) -> new Translation2d[size];
        Logger.recordOutput("TrajectorySim", points.toArray(generator));
    }
}
