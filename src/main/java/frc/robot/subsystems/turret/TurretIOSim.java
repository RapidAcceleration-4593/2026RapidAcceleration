package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.PowerSim;

public class TurretIOSim extends TurretIOReal {

    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

    private final SingleJointedArmSim turret;
    private final DCMotor gearbox;

    public TurretIOSim() {
        gearbox = DCMotor.getNEO(1);

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);

        turret = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(gearbox, kTurretMOI.in(KilogramSquareMeters), kGearRatio),
                gearbox,
                kGearRatio,
                Units.inchesToMeters(10),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        super.updateInputs(inputs);
        updateSimulation();
    }

    private void updateSimulation() {
        turret.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        turret.update(0.02);

        motorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()),
                PowerSim.getRailVoltage(),
                0.02);
        encoderSim.setDistance(Units.radiansToDegrees(turret.getAngleRads()));
        encoderSim.setRate(Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()));

        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
    }
}
