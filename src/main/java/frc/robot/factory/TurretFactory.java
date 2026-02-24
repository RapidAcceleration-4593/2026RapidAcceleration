package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.*;
import frc.robot.util.SimulationManager;

public final class TurretFactory {

    private TurretFactory() {}

    public static TurretSubsystem initialize(SwerveSubsystem swerve) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static TurretSubsystem initializeReal(SwerveSubsystem swerve) {
        return new TurretSubsystem(new TurretIOReal(), swerve::getPose);
    }

    private static TurretSubsystem initializeSim() {
        SimulationManager simulation = SimulationManager.getInstance();
        return new TurretSubsystem(new TurretIOSim(), simulation::getPose);
    }

    private static TurretSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new TurretSubsystem(new TurretIO() {}, swerve::getPose);
    }
}
