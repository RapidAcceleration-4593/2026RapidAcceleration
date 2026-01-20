package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.MAPLESIM_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public final class Simulation {

    private static Simulation instance;

    private final SwerveDriveSimulation simulation;
    private final SimulatedArena arena = SimulatedArena.getInstance();

    private final Pose2d initialPose;

    private final List<IPhysicsSim> sims;

    private Simulation() {
        initialPose = new Pose2d(3, 3, new Rotation2d());
        simulation = new SwerveDriveSimulation(MAPLESIM_CONFIG, initialPose);
        arena.addDriveTrainSimulation(simulation);
        sims = new ArrayList<IPhysicsSim>();
    }

    public static Simulation getInstance() {
        if (kCurrentMode != Mode.SIM) return null;
        if (instance == null) {
            instance = new Simulation();
        }
        return instance;
    }

    public void setPose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

    public Pose2d getPose() {
        return simulation.getSimulatedDriveTrainPose();
    }

    public void resetField() {
        setPose(initialPose);
        arena.resetFieldForAuto();
    }

    public void launchProjectile() {
        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                getPose().getTranslation(),
                kPhysicalOffset.getTranslation(),
                new ChassisSpeeds(), // Consider current robot velocity.
                getPose().getRotation(), // Plus turret rotation.
                Inches.of(20.5),
                MetersPerSecond.of(8.0),
                Degrees.of(75.0));

        projectile.enableBecomesGamePieceOnFieldAfterTouchGround();
        arena.addGamePieceProjectile(projectile);
    }

    public void periodic() {
        for (var sim : sims) {
            sim.updatePlantSim();
        }
        for (var sim : sims) {
            sim.updatePowerSim();
        }
        PowerSim.simulationPeriodic();
        for (var sim : sims) {
            sim.updateIOSim();
        }
        arena.simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", getPose());
        Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    }

    public void addSimulatable(IPhysicsSim sim) {
        sims.add(sim);
    }

    public void removeSimulatable(IPhysicsSim sim) {
        sims.remove(sim);
    }

    public SwerveDriveSimulation raw() {
        return simulation;
    }
}
