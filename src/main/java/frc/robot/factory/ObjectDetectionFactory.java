package frc.robot.factory;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants.*;

import frc.robot.subsystems.vision.objectdetection.*;
import frc.robot.util.SimulationManager;

public final class ObjectDetectionFactory {

    private ObjectDetectionFactory() {}

    public static ObjectDetectionSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static ObjectDetectionSubsystem initializeReal() {
        return new ObjectDetectionSubsystem(new ObjectDetectionIOReal(kCameras[0].name()));
    }

    private static ObjectDetectionSubsystem initializeSim() {
        SimulationManager simulation = SimulationManager.getInstance();
        return new ObjectDetectionSubsystem(new ObjectDetectionIOSim(simulation::getPose, 20));
    }

    private static ObjectDetectionSubsystem initializeReplay() {
        return new ObjectDetectionSubsystem(new ObjectDetectionIO() {});
    }
}
