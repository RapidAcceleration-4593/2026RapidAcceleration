package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.deploy.*;

public final class DeployFactory {

    private DeployFactory() {}

    public static DeploySubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static DeploySubsystem initializeReal() {
        return new DeploySubsystem(new DeployIOReal());
    }

    private static DeploySubsystem initializeSim() {
        return new DeploySubsystem(new DeployIOSim());
    }

    private static DeploySubsystem initializeReplay() {
        return new DeploySubsystem(new DeployIO() {});
    }
}
