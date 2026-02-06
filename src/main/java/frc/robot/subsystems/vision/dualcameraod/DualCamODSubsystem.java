package frc.robot.subsystems.vision.dualcameraod;

import java.util.Arrays;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionInputsAutoLogged;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionIO.TargetObservation;

//A lot of the wanted code/protocols are already in the object detection package

public class DualCamODSubsystem extends SubsystemBase{
	
	//At this point, I have no idea whether this will work or not
	
	private final ObjectDetectionIO ioA, ioB;
	private final ObjectDetectionInputsAutoLogged inputsA, inputsB;
	final SwerveSubsystem swerve;

	/**This code assumes a few other things that may not be true, namely that
	* ioA's camera is, in PositionFromDualYaw, at least, assumed to be on the left.
	* KEEP THIS RULE UP, otherwise we're screwed.
	 */
	public DualCamODSubsystem(ObjectDetectionIO ioA, ObjectDetectionIO ioB, SwerveSubsystem swerve) {
		this.ioA = ioA;
		this.ioB = ioB;
		this.inputsA = new ObjectDetectionInputsAutoLogged();
		this.inputsB = new ObjectDetectionInputsAutoLogged();
		this.swerve = swerve;
	}
	
	@Override
	public void periodic(){
		ioA.updateInputs(inputsA);
		ioB.updateInputs(inputsB);
	}

	public void iterateInputs(){
		Stream<TargetObservation> bInputStream = Arrays.stream(inputsA.latestTargets);
		//We don't want a single timestamp, but we need a way to split the poses we get into separate time stamps
		//Ideally, we'd want to lower the timestamped data we store based on how long ago it was - the farther away, the less we need to keep
		//otherwise, we filter each by what isn't on the edge of the cameras, sort by pitch, then yaw, and, from there, it's pretty easy to match targets and run positionfromyaw :)
	}

	/**This function uses the yaw of two cameras to determine the position of a target.
	 * Right now, it returns a Pose2D, though we should eventually edit it to return a Pose3D.
	 * TODO: right now, it requires both cameras to be exactly parallel, change that, I'm too lazy tonight
	 * other TODO: currently assumes that targets were taken at exact moment of calculation, which they very much were not)
	*/
	public Pose2d positionFromDualYaw (ObjectDetectionIO.TargetObservation a, ObjectDetectionIO.TargetObservation b){
		double distanceBetweenCams = DualCamODConstants.kSyncableCameras[0].robotToCamera().getX() - DualCamODConstants.kSyncableCameras[1].robotToCamera().getX();
		double unscaledDistance = (Math.cos(b.yaw().getRadians()) * Math.sin(a.yaw().getRadians())) / (Math.sin(b.yaw().getRadians())) + Math.cos(a.yaw().getRadians());
		double scaleFactor = distanceBetweenCams/unscaledDistance;
		//transform our distances from robot center based on robot pose
		Pose2d posFromRobot = new Pose2d(
			-(distanceBetweenCams/2 /*center*/ - Math.cos(a.yaw().getRadians()) * scaleFactor /*left edge*/),
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