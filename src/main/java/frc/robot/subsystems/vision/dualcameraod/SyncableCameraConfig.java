package frc.robot.subsystems.vision.dualcameraod;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO;

public class SyncableCameraConfig {
	String name; Transform3d robotToCamera; double yawEdge, pitchEdge;

	/**Adjusts X based on angle for sine and cosine functions
	 * Allows positionFromDualYaw to work without cameras being parallel
	 * @param otra Other robot (Otra means other in Spanish)
	*/
	public double getAdjustedRadians(ObjectDetectionIO.TargetObservation observation){
		double ourAngle = this.robotToCamera.getRotation().getMeasureZ().in(Radians);
		return ourAngle + observation.yaw().getRadians();
	}
	public double getAdjustedX(SyncableCameraConfig otra, ObjectDetectionIO.TargetObservation observation){
		double yDifference = otra.getRobotToCamera().getY() - this.robotToCamera.getY();
		double scaleFactor = yDifference / -Math.cos(this.getAdjustedRadians(observation));
		return this.robotToCamera.getX() - scaleFactor * Math.sin(this.getAdjustedRadians(observation)); //very temporary, line only written 
	}
	public Transform3d getRobotToCamera(){
		return robotToCamera;
	}
	public double getPitchEdge(){
		return this.pitchEdge;
	}
	public double getYawEdge(){
		return this.yawEdge;
	}
}
