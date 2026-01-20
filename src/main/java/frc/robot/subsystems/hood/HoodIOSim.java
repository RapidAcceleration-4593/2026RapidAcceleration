package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.util.PowerSim;

public class HoodIOSim extends HoodIOReal {

    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;
    private final DIOSim lsSim;

    private final DCMotor gearbox;

    public HoodIOSim() {
        gearbox = DCMotor.getNeo550(1);

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);
        lsSim = new DIOSim(limitSwitch);
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        super.updateInputs(inputs);
        updateSimulation();
    }

    /** Updates simulation variables periodically. */
    private void updateSimulation() {
        encoderSim.setRate(motorSim.getVelocity() / kMotorToEncoderGearing);
        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
    }
}
