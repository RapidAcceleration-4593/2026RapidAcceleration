package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.SimulationManager;

public final class HoodFactory {

    private HoodFactory() {}

    public static HoodSubsystem initialize(SwerveSubsystem swerve) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static HoodSubsystem initializeReal(SwerveSubsystem swerve) {
        return new HoodSubsystem(new HoodIOReal(), swerve::getPose);
    }

    private static HoodSubsystem initializeSim() {
        SimulationManager simulation = SimulationManager.getInstance();
        return new HoodSubsystem(new HoodIOSim(), simulation::getPose);
    }

    private static HoodSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new HoodSubsystem(new HoodIO() {}, swerve::getPose);
    }
}
