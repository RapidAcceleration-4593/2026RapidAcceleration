package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;

public class TurretIOSim extends TurretIOReal implements IPhysicsSim {

    private final SingleJointedArmSim turret;

    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

    public TurretIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);

        turret = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(gearbox, kTurretMOI.in(KilogramSquareMeters), kGearRatio),
                gearbox,
                kGearRatio,
                Units.inchesToMeters(10),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);
    }

    @Override
    public void updatePlantSim() {
        turret.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts));
        turret.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(motorSim.getMotorCurrent()));
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()),
                PowerSim.getRailVoltage().in(Volts),
                0.02);
        encoderSim.setDistance(Units.radiansToDegrees(turret.getAngleRads()));
        encoderSim.setRate(Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()));
    }
}
