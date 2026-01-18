package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Simulation;

public final class HoodFactory {

    private HoodFactory() {}

    public static HoodSubsystem initialize(SwerveSubsystem swerve) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim(swerve);
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static HoodSubsystem initializeReal(SwerveSubsystem swerve) {
        return new HoodSubsystem(new HoodIOReal(), swerve::getPose);
    }

    private static HoodSubsystem initializeSim(SwerveSubsystem swerve) {
        return new HoodSubsystem(new HoodIOSim(), Simulation.getInstance()::getPose);
    }

    private static HoodSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new HoodSubsystem(new HoodIO() {}, swerve::getPose);
    }
}
