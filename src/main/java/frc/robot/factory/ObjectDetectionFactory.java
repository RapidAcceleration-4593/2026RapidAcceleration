package frc.robot.factory;

import static frc.robot.Constants.currentMode;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.objectdetection.*;
import frc.robot.util.Simulation;

public final class ObjectDetectionFactory {

    private ObjectDetectionFactory() {}

    public static ObjectDetectionSubsystem initialize(SwerveSubsystem swerve) {
        return switch (currentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim(swerve);
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static ObjectDetectionSubsystem initializeReal(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(new ObjectDetectionIOReal(), swerve::getPose);
    }

    private static ObjectDetectionSubsystem initializeSim(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(
                new ObjectDetectionIOSim(Simulation.getInstance()::getPose), Simulation.getInstance()::getPose);
    }

    private static ObjectDetectionSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(new ObjectDetectionIO() {}, swerve::getPose);
    }
}
