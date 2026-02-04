package frc.robot.subsystems.vision.dualcameraod;

import edu.wpi.first.math.geometry.Transform3d;

//I suspect that I'm writing straight garbage...

public final class DualCamODConstants {
	//Declare these terms when we know what they are
	public record SyncableCameraConfig(String name, Transform3d robotToCamera, double yawEdge, double pitchEdge) {}
	//yawedge and pitchedge refer to parts that won't show in other camera views
	public static final SyncableCameraConfig[] kSyncableCameras = {};
	//again, add cameras once we know what we're doing, which I currently really don't
}
