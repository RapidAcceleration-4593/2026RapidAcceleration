package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.robot.subsystems.swerve.SwerveConstants.kRobotMOI;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim extends TurretIOReal {

    private DCMotor gearbox = DCMotor.getNEO(1);
	private SparkMaxSim motorSim = new SparkMaxSim(kTurretMotor, gearbox);
    private SingleJointedArmSim turretSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(gearbox, kRobotMOI.in(KilogramSquareMeters), ));
	private EncoderSim encoderSim = new EncoderSim(kTurretEncoder);
		
    @Override
    public void updateInputs(TurretInputs inputs) {
        updateSimulation();
        super.updateInputs(inputs);
    }

    private void updateSimulation() {
	}

    private double calculateGearing() {
        return kGearboxRatio * kRingGear / kDriveGear;
    }
}
