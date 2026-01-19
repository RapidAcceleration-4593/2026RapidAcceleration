package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.intake.*;
import frc.robot.util.Simulation;

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
        return new IntakeSubsystem(new IntakeIOSim(Simulation.getInstance().raw()));
    }

    private static IntakeSubsystem initializeReplay() {
        return new IntakeSubsystem(new IntakeIO() {});
    }
}
