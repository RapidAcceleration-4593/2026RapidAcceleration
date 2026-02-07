package frc.robot.subsystems.vision.dualcameraod;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO;

public class SyncableCameraConfig {
	String name; Transform3d robotToCamera; double yawEdge, pitchEdge;

	/**Adjusts X based on angle for sine and cosine functions
	 * Allows positionFromDualYaw to work without cameras being parallel
	 * @param otra Other robot (Otra means other in Spanish)
	*/
	public double getAdjustedX(SyncableCameraConfig otra, ObjectDetectionIO.TargetObservation observation){
		double yDifference = otra.getRobotToCamera().getY() - this.robotToCamera.getY();
		double scaleFactor = yDifference / -Math.cos(observation.yaw().getRadians());
		return this.robotToCamera.getX() - scaleFactor * Math.sin(observation.yaw().getRadians()); //very temporary, line only written 
	}
	public Transform3d getRobotToCamera(){
		return robotToCamera;
	}
}
