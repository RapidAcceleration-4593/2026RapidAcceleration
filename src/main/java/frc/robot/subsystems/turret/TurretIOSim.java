package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class TurretIOSim extends TurretIOReal implements IPhysicsSim {

    private final SingleJointedArmSim turretSim;

    private final SparkMaxSim motorSim;
    private final SparkAbsoluteEncoderSim encoderSim;

    public TurretIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);

        turretSim = new SingleJointedArmSim(
                gearbox,
                kMotorToTurretGearing,
                kTurretMOI.in(KilogramSquareMeters),
                Units.inchesToMeters(10),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkAbsoluteEncoderSim(motor);
    }

    @Override
    public void updatePlantSim() {
        turretSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        turretSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(motorSim.getMotorCurrent());
    }

    @Override
    public void updateIOSim() {
        AngularVelocity turretVelocity = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());
        AngularVelocity motorVelocity = turretVelocity.times(kMotorToTurretGearing);

        motorSim.iterate(
                motorVelocity.in(RPM), SimulatedBattery.getBatteryVoltage().in(Volts), 0.02);
        encoderSim.setPosition(Degrees.convertFrom(turretSim.getAngleRads(), Radians));
    }
}
