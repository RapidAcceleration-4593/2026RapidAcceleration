package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.PowerSim;

public class SpindexerIOSim implements SpindexerIO {

    private final SparkMax spindexer;
    private final SparkMax feeder;

    private final SparkMaxSim spindexerSim;
    private final SparkMaxSim feederSim;

    private final SparkMaxConfig config;

    public SpindexerIOSim() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        spindexer = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        feeder = new SparkMax(kFeederMotorID, MotorType.kBrushless);

        spindexerSim = new SparkMaxSim(spindexer, DCMotor.getNEO(1));
        feederSim = new SparkMaxSim(feeder, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(SpindexerInputs inputs) {
        updateSimulation();

        inputs.spindexerVelocity = RPM.of(spindexerSim.getVelocity());
        inputs.feederVelocity = RPM.of(feederSim.getVelocity());

        inputs.spindexerVolts = Volts.of(spindexerSim.getAppliedOutput() * PowerSim.getRailVoltage());
        inputs.feederVolts = Volts.of(feederSim.getAppliedOutput() * PowerSim.getRailVoltage());

        inputs.spindexerCurrent = Amps.of(spindexerSim.getMotorCurrent());
        inputs.feederCurrent = Amps.of(feederSim.getMotorCurrent());
    }

    private void updateSimulation() {
        PowerSim.addCurrentDraw(spindexerSim.getMotorCurrent());
        PowerSim.addCurrentDraw(feederSim.getMotorCurrent());
    }

    @Override
    public void setSpindexerSpeed(double speed) {
        spindexer.set(speed);
    }

    @Override
    public void stopSpindexer() {
        spindexer.stopMotor();
    }

    @Override
    public void setFeederSpeed(double speed) {
        feeder.set(speed);
    }

    @Override
    public void stopFeeder() {
        feeder.stopMotor();
    }
}
