package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.deploy.*;

public final class DeployFactory {

    private DeployFactory() {}

    public static DeploySubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new DeploySubsystem(new DeployIOReal());
            case SIM -> new DeploySubsystem(new DeployIOSim());
            case REPLAY -> new DeploySubsystem(new DeployIO() {});
        };
    }
}
