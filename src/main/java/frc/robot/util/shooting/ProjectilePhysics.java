package frc.robot.util.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;
import org.littletonrobotics.junction.Logger;

/**
 * A class wrapping the calculations concerning the dynamics of projectiles, including drag.
 *
 * <p>Drag is modeled as quadratic (F_drag = -k|v|v, k = 0.5 * rho * Cd * A), which couples the x and y axes through the
 * total speed. There is no closed-form solution, so positions and velocities are computed with a 4th-order Runge-Kutta
 * (RK4) integrator.
 */
public class ProjectilePhysics {
    private String name;
    private double projectileMassKg;
    private double dragConstant;

    /** RK4 internal time-step in seconds. Smaller = more accurate but slower. */
    private double rk4Step = 0.1;

    private static double gravity = 9.8; // Meters per second squared
    private static double airDensity = 1.225; // Kg per cubic meter
    private static IntFunction<Translation2d[]> arrayGenerator = (size) -> new Translation2d[size];

    public static final ProjectilePhysics kFuelPhysics = new ProjectilePhysics("Fuel", 0.2257, 0.01767, 0.47, 0.1);

    public ProjectilePhysics(
            String name, double projectileMassKg, double crossSectAreaSqMeter, double dragCoefficient, double rk4Step) {
        this.projectileMassKg = projectileMassKg;
        this.rk4Step = rk4Step;
        this.name = name;
        dragConstant = 0.5 * airDensity * dragCoefficient * crossSectAreaSqMeter;
    }

    public String getName() {
        return name;
    }

    /**
     * State vector layout: [x, vx, y, vy]
     *
     * <p>Returns the time-derivative of the state under quadratic drag and gravity.
     */
    private double[] derivatives(double[] s) {
        double vx = s[1];
        double vy = s[3];
        double speed = Math.sqrt(vx * vx + vy * vy);
        double ax = -(dragConstant / projectileMassKg) * speed * vx;
        double ay = -gravity - (dragConstant / projectileMassKg) * speed * vy;
        return new double[] {vx, ax, vy, ay};
    }

    /** Advances the state vector by {@code dt} seconds using a single RK4 step. */
    private double[] rk4Step(double[] s, double dt) {
        double[] k1 = scaleArr(derivatives(s), dt);
        double[] k2 = scaleArr(derivatives(addArr(s, scaleArr(k1, 0.5))), dt);
        double[] k3 = scaleArr(derivatives(addArr(s, scaleArr(k2, 0.5))), dt);
        double[] k4 = scaleArr(derivatives(addArr(s, k3)), dt);

        double[] result = new double[4];
        for (int i = 0; i < 4; i++) {
            result[i] = s[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }
        return result;
    }

	/** Advances the {@code currentState} vector by {@code timeStep} and returns the new vector. */
    private double[] integrateState(double[] currentState, double timeStep) {
        assert currentState.length == 4 : "The state array must have a length of 4.";
        double[] state = currentState;
        double t = 0;
        while (t < timeStep) {
            double dt = Math.min(rk4Step, timeStep - t);
            state = rk4Step(state, dt);
            t += dt;
        }
        return state;
    }

    /**
     * Calculates the flight trajectory of the projectile, finishing when the y position of the projectile falls beneath
     * 0.
     *
     * @param initialPosition The initial position of the projectile at time 0.
     * @param angle The angle the projectile is launched at.
     * @param initialSpeed The initial speed the projectile is launched with.
     * @param timestep The amount of time that should pass in between each point which is logged.
     * @return A {@link List} of {@link Translation2d}s which represent the position of the projectile at each
     *     successive timestep, starting at time 0 and finishing with the final value with a positive y value.
     */
    public List<Translation2d> calculateTrajectory(
            Translation2d initialPosition, double angle, double initialSpeed, double timestep) {
        List<Translation2d> points = new ArrayList<>();
        double initialVelocityX = initialSpeed * Math.cos(angle);
        double initialVelocityY = initialSpeed * Math.sin(angle);

        double[] state =
                new double[] {initialPosition.getX(), initialVelocityX, initialPosition.getY(), initialVelocityY};
        points.add(new Translation2d(state[0], state[2]));

        while (state[2] >= 0) {
            state = integrateState(state, timestep);
            points.add(new Translation2d(state[0], state[2]));
        }

        return points;
    }

    public void runSimulation() {
        var points = calculateTrajectory(Translation2d.kZero, Units.degreesToRadians(45), 8, 0.1);
        Logger.recordOutput(name + "TrajectorySim", points.toArray(arrayGenerator));
    }

    private static double[] addArr(double[] a, double[] b) {
        double[] r = new double[a.length];
        for (int i = 0; i < a.length; i++) r[i] = a[i] + b[i];
        return r;
    }

    private static double[] scaleArr(double[] a, double s) {
        double[] r = new double[a.length];
        for (int i = 0; i < a.length; i++) r[i] = a[i] * s;
        return r;
    }
}
