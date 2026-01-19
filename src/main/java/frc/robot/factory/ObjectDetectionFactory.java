package frc.robot.factory;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants.*;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.objectdetection.*;
import frc.robot.util.Simulation;

public final class ObjectDetectionFactory {

    private ObjectDetectionFactory() {}

    public static ObjectDetectionSubsystem initialize(SwerveSubsystem swerve) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim(swerve);
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static ObjectDetectionSubsystem initializeReal(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(new ObjectDetectionIOReal(kCameras[0].name()));
    }

    private static ObjectDetectionSubsystem initializeSim(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(new ObjectDetectionIOSim(Simulation.getInstance()::getPose, 20));
    }

    private static ObjectDetectionSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new ObjectDetectionSubsystem(new ObjectDetectionIO() {});
    }
}
