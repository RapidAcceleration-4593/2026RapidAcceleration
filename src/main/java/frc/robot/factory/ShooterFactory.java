package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.shooter.*;

public final class ShooterFactory {

    private ShooterFactory() {}

    public static ShooterSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static ShooterSubsystem initializeReal() {
        return new ShooterSubsystem(new ShooterIOReal());
    }

    private static ShooterSubsystem initializeSim() {
        return new ShooterSubsystem(new ShooterIOSim());
    }

    private static ShooterSubsystem initializeReplay() {
        return new ShooterSubsystem(new ShooterIO() {});
    }
}
