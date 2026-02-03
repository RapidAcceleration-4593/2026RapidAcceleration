package frc.robot.subsystems.vision.dualcameraod;

import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO;

//A lot of the wanted code/protocols are already in the object detection package

public class DualCamODSubsystem {
	
	//At this point, I have no idea whether this will work or not
	
	private final ObjectDetectionIO ioA;
	private final ObjectDetectionIO ioB;

	public DualCamODSubsystem(ObjectDetectionIO ioA, ObjectDetectionIO ioB) {
		this.ioA = ioA;
		this.ioB = ioB;
	}

	/*What we know we want
		1. A method I already know that turns two targets into one target w/ info stored as a Pose3D...
		We'll write it as a Pose2D for now
	*/

	public /*possibly change that */ Pose2d positionFromDualYaw (ObjectDetectionIO.TargetObservation a, ObjectDetectionIO.TargetObservation b){
		double distanceBetweenCams = KSyncableCameras[0].cameraToFrontCenter.getX() - KSyncableCameras[1].cameraToFrontCenter.getX();
		double unscaledDistance = (Math.cos(b.yaw().getRadians()) * Math.sin(a.yaw().getRadians())) / (Math.sin(b.yaw().getRadians())) + Math.cos(a.yaw().getRadians());
		//equation seems wrong, check this later
		double scaleFactor = distanceBetweenCams/unscaledDistance;
	}


	/*What I'm scared about
		1. Will the robot have performance issues due to large amounts of complex calcs?
	*/
}