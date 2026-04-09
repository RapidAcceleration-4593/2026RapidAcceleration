package frc.robot.subsystems.vision.quest;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {

    @AutoLog
    public static class QuestNavInputs {
        public boolean connected = false;
        public boolean tracking = false;
        public Time latency = Milliseconds.zero();
        public double batteryPercent = 0.0;
        public long trackingLostCount = 0;

        public PoseFrameLog[] unreadFrames = new PoseFrameLog[0];
    }

    public record PoseFrameLog(Pose3d questPose, double timestamp, boolean isTracking) {}

    public default void updateInputs(QuestNavInputs inputs) {}

    public default void setPose(Pose3d questPose) {}
}
