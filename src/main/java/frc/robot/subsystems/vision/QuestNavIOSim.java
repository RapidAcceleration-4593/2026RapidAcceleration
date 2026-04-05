package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class QuestNavIOSim implements QuestNavIO {

    @Override
    public void updateInputs(QuestNavInputs inputs) {
        inputs.connected = false;
        inputs.unreadFrames = new PoseFrameLog[0];
    }

    @Override
    public void setPose(Pose3d questPose) {}
}
