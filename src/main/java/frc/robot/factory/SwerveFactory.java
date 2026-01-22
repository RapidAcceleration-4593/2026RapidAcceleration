package frc.robot.factory;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import frc.robot.subsystems.swerve.*;
import frc.robot.util.SimulationManager;

public final class SwerveFactory {

    private SwerveFactory() {}

    public static SwerveSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static SwerveSubsystem initializeReal() {
        return new SwerveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(FrontLeft),
                new ModuleIOTalonFXReal(FrontRight),
                new ModuleIOTalonFXReal(BackLeft),
                new ModuleIOTalonFXReal(BackRight),
                pose -> {});
    }

    private static SwerveSubsystem initializeSim() {
        SimulationManager simulation = SimulationManager.getInstance();
        var maplesim = simulation.getDriveSimulation();
        var modules = maplesim.getModules();

        return new SwerveSubsystem(
                new GyroIOSim(maplesim.getGyroSimulation()),
                new ModuleIOTalonFXSim(FrontLeft, modules[0]),
                new ModuleIOTalonFXSim(FrontRight, modules[1]),
                new ModuleIOTalonFXSim(BackLeft, modules[2]),
                new ModuleIOTalonFXSim(BackRight, modules[3]),
                simulation::setPose);
    }

    private static SwerveSubsystem initializeReplay() {
        return new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                pose -> {});
    }
}
