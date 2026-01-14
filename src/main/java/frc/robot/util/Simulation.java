package frc.robot.util;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public final class Simulation {

    private static Simulation instance;

    private final SwerveDriveSimulation simulation;
    private final SimulatedArena arena = SimulatedArena.getInstance();

    private final Pose2d initialPose;

    private Simulation() {
        initialPose = new Pose2d(3, 3, new Rotation2d());
        simulation = new SwerveDriveSimulation(SwerveSubsystem.mapleSimConfig, initialPose);
        arena.addDriveTrainSimulation(simulation);
    }

    public static Simulation getInstance() {
        if (currentMode != Mode.SIM) return null;
        if (instance == null) {
            instance = new Simulation();
        }
        return instance;
    }

    public Pose2d getPose() {
        return simulation.getSimulatedDriveTrainPose();
    }

    public void setPose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

    public void resetField() {
        setPose(initialPose);
        arena.resetFieldForAuto();
    }

    public void periodic() {
        arena.simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", getPose());
        Logger.recordOutput("FieldSimulation/Coral", arena.getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae", arena.getGamePiecesArrayByType("Algae"));
    }

    public SwerveDriveSimulation raw() {
        return simulation;
    }
}
