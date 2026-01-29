package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class HoodIOSim extends HoodIOReal implements IPhysicsSim {

    private final SingleJointedArmSim hoodSim;
    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

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
        encoderSim = new EncoderSim(encoder);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        hoodSim.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts));
        hoodSim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(hoodSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateIOSim() {
        AngularVelocity motorVelocity =
                RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(kMotorToHoodGearing);
        motorSim.iterate(motorVelocity.in(RPM), PowerSim.getRailVoltage().in(Volts), 0.02);
        encoderSim.setDistance(Degrees.convertFrom(hoodSim.getAngleRads(), Radians));
    }
}
