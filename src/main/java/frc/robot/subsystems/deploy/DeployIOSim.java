package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class DeployIOSim extends DeployIOReal implements IPhysicsSim {

    private final SingleJointedArmSim deploySim;

    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

    public DeployIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(2);

        deploySim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        gearbox, kMOI.in(KilogramSquareMeters), kMotorToDeployGearing),
                gearbox,
                kMotorToDeployGearing,
                Units.inchesToMeters(12.0),
                0.0,
                0.0,
                false,
                0.0);

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        double input = motorSim.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts);
        deploySim.setInput(input);
        deploySim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(motorSim.getMotorCurrent()));
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(0.0, PowerSim.getRailVoltage().in(Volts), 0.02); // TODO: For Flanegan, love Lincoln
        encoderSim.setDistance(0.0); // TODO: For Flanegan, love Lincoln
    }
}
