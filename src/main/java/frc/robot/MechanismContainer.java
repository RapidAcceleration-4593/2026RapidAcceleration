package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.mechanism.Axis;
import frc.robot.util.mechanism.MechanismLigament3D;
import frc.robot.util.mechanism.Node3D;
import frc.robot.util.mechanism.Robot3D;

public class MechanismContainer {
    public MechanismContainer() {

        Node3D intakeNode = new Node3D("Intake", Pose3d.kZero);
        Node3D spindexerNode = new Node3D("Spindexer", new Pose3d(0.034, 0, 0, Rotation3d.kZero));
        Node3D shooterBaseNode = new Node3D("ShooterBase", new Pose3d(-0.144, 0, 0, Rotation3d.kZero));
        Node3D hoodNode = new Node3D("Hood", new Pose3d(0.125, 0, 0.47, new Rotation3d(0, 0.25, 0)), shooterBaseNode);
        Node3D climberNode = new Node3D("Climber", Pose3d.kZero);

        MechanismLigament3D intake = new MechanismLigament3D(intakeNode, Axis.X);
        MechanismLigament3D spindexer = new MechanismLigament3D(spindexerNode, Axis.Z);
        MechanismLigament3D shooterBase = new MechanismLigament3D(shooterBaseNode, Axis.Z);
        MechanismLigament3D hood = new MechanismLigament3D(hoodNode, Axis.Y);
        MechanismLigament3D climber = new MechanismLigament3D(climberNode, Axis.Z);

        Robot3D.getInstance().addMechanisms(new MechanismLigament3D[] {intake, spindexer, shooterBase, hood, climber});
    }
}
