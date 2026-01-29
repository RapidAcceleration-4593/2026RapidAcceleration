package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.intake.*;
import frc.robot.util.SimulationManager;

public final class IntakeFactory {

    private IntakeFactory() {}

    public static IntakeSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static IntakeSubsystem initializeReal() {
        return new IntakeSubsystem(new IntakeIOReal());
    }

    private static IntakeSubsystem initializeSim() {
        SimulationManager simulation = SimulationManager.getInstance();
        return new IntakeSubsystem(new IntakeIOSim(simulation.getDriveSimulation()));
    }

    private static IntakeSubsystem initializeReplay() {
        return new IntakeSubsystem(new IntakeIO() {});
    }
}
