package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.MAPLESIM_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public final class SimulationManager {

    private static SimulationManager instance;

    private final SwerveDriveSimulation simulation;
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final List<IPhysicsSim> components;

    private final Pose2d initialPose;

    private SimulationManager() {
        initialPose = new Pose2d(Meters.of(3.0), Meters.of(3.0), new Rotation2d());
        simulation = new SwerveDriveSimulation(MAPLESIM_CONFIG, initialPose);
        arena.addDriveTrainSimulation(simulation);
        components = new ArrayList<>();
    }

    /** Retrieves the SimulationManager instance during simulation. */
    public static SimulationManager getInstance() {
        if (kCurrentMode != Mode.SIM) return null;
        if (instance == null) {
            instance = new SimulationManager();
        }
        return instance;
    }

    /** Sets the simulated robot pose. */
    public void setPose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

    /** Retrieves the current simulated robot pose. */
    public Pose2d getPose() {
        return simulation.getSimulatedDriveTrainPose();
    }

    /** Resets the robot and field state for autonomous. */
    public void resetField() {
        setPose(initialPose);
        arena.resetFieldForAuto();
    }

    /** Simulates an object being launched from the shooter mechanism. */
    public void launchProjectile(Angle angle, AngularVelocity velocity) {
        Distance topWheelRadius = Inches.of(1.25);
        Distance botWheelRadius = Inches.of(2.0);

        LinearVelocity topLinearVelocity =
                MetersPerSecond.of(velocity.in(RadiansPerSecond) * topWheelRadius.in(Meters));
        LinearVelocity botLinearVelocity =
                MetersPerSecond.of(velocity.in(RadiansPerSecond) * botWheelRadius.in(Meters));

        LinearVelocity totalLinearVelocity =
                topLinearVelocity.plus(botLinearVelocity).div(2);

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                getPose().getTranslation(),
                kPhysicalOffset.getTranslation(),
                new ChassisSpeeds(),
                getPose().getRotation(), // Plus turret rotation.
                Inches.of(20.5),
                totalLinearVelocity,
                Degrees.of(90.0).minus(angle));

        arena.addGamePieceProjectile(projectile);
    }

    /** Run periodically during simulation. */
    public void periodic() {
        for (var component : components) {
            component.updatePlantSim();
        }

        for (var component : components) {
            component.updatePowerSim();
        }

        PowerSim.simulationPeriodic();
        for (var component : components) {
            component.updateIOSim();
        }

        arena.simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", getPose());
        Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    }

    /** Registers a subsystem physics simulation. */
    public void addSimulatable(IPhysicsSim sim) {
        components.add(sim);
    }

    /** Removes a subsystem physics simulation. */
    public void removeSimulatable(IPhysicsSim sim) {
        components.remove(sim);
    }

    /** Retrieves the raw MapleSim drivetrain simulation. */
    public SwerveDriveSimulation getDriveSimulation() {
        return simulation;
    }
}
