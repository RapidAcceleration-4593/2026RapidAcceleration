package frc.robot.subsystems.vision.dualcameraod;

import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO;

//A lot of the wanted code/protocols are already in the object detection package

public class DualCamODSubsystem {
	
	//At this point, I have no idea whether this will work or not
	
	private final ObjectDetectionIO ioA;
	private final ObjectDetectionIO ioB;
	final SwerveSubsystem swerve;

	public DualCamODSubsystem(ObjectDetectionIO ioA, ObjectDetectionIO ioB, SwerveSubsystem swerve) {
		this.ioA = ioA;
		this.ioB = ioB;
		this.swerve = swerve;
	}

	/**This function uses the yaw of two cameras to determine the position of a target.
	 * Right now, it returns a Pose2D, though we should eventually edit it to return a Pose3D.
	 * TODO: right now, it requires both cameras to be exactly parallel, change that, I'm too lazy tonight)
	*/
	public Pose2d positionFromDualYaw (ObjectDetectionIO.TargetObservation a, ObjectDetectionIO.TargetObservation b){
		double distanceBetweenCams = DualCamODConstants.kSyncableCameras[0].robotToCamera().getX() - DualCamODConstants.kSyncableCameras[1].robotToCamera().getX();
		double unscaledDistance = (Math.cos(b.yaw().getRadians()) * Math.sin(a.yaw().getRadians())) / (Math.sin(b.yaw().getRadians())) + Math.cos(a.yaw().getRadians());
		double scaleFactor = distanceBetweenCams/unscaledDistance;
		//transform our distances from robot center based on robot pose
		Pose2d posFromRobot = new Pose2d(
			-(distanceBetweenCams/2 - Math.cos(a.yaw().getRadians()) * scaleFactor),
			Math.sin(a.yaw().getRadians()) * scaleFactor + DualCamODConstants.kSyncableCameras[0].robotToCamera().getY(),
			Rotation2d.fromDegrees(0)
			);
		//now that it's robot to camera, I need to add that y
		Transform2d absolutePose = new Transform2d(new Pose2d(), this.swerve.getPose());
		return posFromRobot.transformBy(absolutePose);
	}

	//Ideally, if I understand gaussian distribution well enough, I could use that to figure out whether our measures are behaving under Kalman principles
	//... or if they're moving

	/*What I'm scared about
		1. Will the robot have performance issues due to large amounts of complex calcs?
	*/
}