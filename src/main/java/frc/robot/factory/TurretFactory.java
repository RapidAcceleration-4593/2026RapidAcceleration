package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.turret.*;

public final class TurretFactory {

    private TurretFactory() {}

    public static TurretSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static TurretSubsystem initializeReal() {
        return new TurretSubsystem(new TurretIOReal());
    }

    private static TurretSubsystem initializeSim() {
        return new TurretSubsystem(new TurretIOSim());
    }

    private static TurretSubsystem initializeReplay() {
        return new TurretSubsystem(new TurretIO() {});
    }
}
