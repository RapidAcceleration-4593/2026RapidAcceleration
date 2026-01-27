package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

public class AngularMechanism3D extends Mechanism3D {
	private Axis axis;

	public AngularMechanism3D(String name, Pose3d offset, Axis axis) {
		super(name, offset);
		this.axis = axis;
	}

	public AngularMechanism3D(String name, Pose3d offset, Mechanism3D parent, Axis axis) {
		super(name, offset, parent);
		this.axis = axis;
	}

	public void setAngle(Angle angle) {
		switch (axis) {
			case X: this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(angle.in(Radians), 0, 0));
			break;
			case Y: this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(0, angle.in(Radians), 0));
			break;
			case Z: this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, angle.in(Radians)));
			break;
		}
	}
}
