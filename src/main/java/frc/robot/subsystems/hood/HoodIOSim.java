package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class HoodIOSim extends HoodIOReal implements IPhysicsSim {

    private final SingleJointedArmSim hoodSim;

    private final SparkMaxSim motorSim;
    private final SparkAbsoluteEncoderSim encoderSim;
    private final DIOSim limitSwitchSim;

    public HoodIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);

        hoodSim = new SingleJointedArmSim(
                gearbox,
                kMotorToHoodGearing,
                kHoodMOI.in(KilogramSquareMeters),
                Units.inchesToMeters(12.0),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                true,
                kMinimumAngle.in(Radians));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkAbsoluteEncoderSim(motor);
        limitSwitchSim = new DIOSim(limitswitch);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        hoodSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        hoodSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(hoodSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        AngularVelocity hoodVelocity = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());
        AngularVelocity encoderVelocity = hoodVelocity.times(kEncoderToHoodGearing);
        AngularVelocity motorVelocity = encoderVelocity.times(kMotorToEncoderGearing);

        motorSim.iterate(
                motorVelocity.in(RPM), SimulatedBattery.getBatteryVoltage().in(Volts), 0.02);
        encoderSim.setPosition(Degrees.convertFrom(hoodSim.getAngleRads(), Radians));
        limitSwitchSim.setValue(hoodSim.hasHitLowerLimit());
    }
}
