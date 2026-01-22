package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.climber.*;

public final class ClimberFactory {

    private ClimberFactory() {}

    public static ClimberSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static ClimberSubsystem initializeReal() {
        return new ClimberSubsystem(new ClimberIOReal());
    }

    private static ClimberSubsystem initializeSim() {
        return new ClimberSubsystem(new ClimberIOSim());
    }

    private static ClimberSubsystem initializeReplay() {
        return new ClimberSubsystem(new ClimberIO() {});
    }
}
