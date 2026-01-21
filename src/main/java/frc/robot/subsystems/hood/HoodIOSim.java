package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.Simulation;

public class HoodIOSim extends HoodIOReal implements IPhysicsSim {

    private final SingleJointedArmSim hoodSim;
    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

    private final DCMotor gearbox;

    public HoodIOSim() {
        gearbox = DCMotor.getNeo550(1);

        hoodSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        gearbox, kHoodMOI.in(KilogramSquareMeters), kMotorToHoodGearing),
                gearbox,
                kMotorToHoodGearing,
                0.3,
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                true,
                kMinimumAngle.in(Radians));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);

        Simulation.getInstance().addSimulatable(this);
    }

    public void updateInputs(HoodInputs inputs) {
        super.updateInputs(inputs);
    }

    @Override
    public void updatePlantSim() {
        hoodSim.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        hoodSim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(hoodSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        var motorRPM = Units.radiansPerSecondToRotationsPerMinute(hoodSim.getVelocityRadPerSec()) * kMotorToHoodGearing;
        motorSim.iterate(motorRPM, PowerSim.getRailVoltage(), 0.02);
        encoderSim.setDistance(Units.radiansToDegrees(hoodSim.getAngleRads()));
    }
}
