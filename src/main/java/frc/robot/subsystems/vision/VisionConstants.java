package frc.robot.subsystems.vision;

public class VisionConstants {
    public static final String kSelectedVisionNTAddress = "AccelerationStation/SelectedVisionSystem";

    public enum VisionSystems {
        Apriltag("OrangePi"),
        Quest("Oculus");

        public String name;

        VisionSystems(String name) {
            this.name = name;
        }
    }
}
