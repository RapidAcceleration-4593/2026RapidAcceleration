package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PowerSim;

public class TurretIOSim implements TurretIO {

    private final SingleJointedArmSim turretSim;
    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;
    private final DCMotor gearbox;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d turretLigament;

    public TurretIOSim() {
        this.gearbox = DCMotor.getNEO(1);

        this.motorSim = new SparkMaxSim(kTurretMotor, gearbox);
        this.encoderSim = new EncoderSim(kTurretEncoder);

        this.turretSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        gearbox, kTurretMOI.in(KilogramSquareMeters), kMotorTurretGearing),
                gearbox,
                kMotorTurretGearing,
                0.5,
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        mechanism = new Mechanism2d(2, 2);
        MechanismRoot2d root = mechanism.getRoot("TurretRoot", 1, 1);

        turretLigament = root.append(new MechanismLigament2d("Turret", 0.6, 0));
        SmartDashboard.putData("Turret/Mechanism", mechanism);
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        updateSimulation();

        inputs.angle = Degrees.of(encoderSim.getDistance());
        inputs.angularVelocity = DegreesPerSecond.of(encoderSim.getRate());
    }

    private void updateSimulation() {
        turretSim.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        turretSim.update(0.02); // 20 milliseconds.
        motorSim.iterate(turretSim.getVelocityRadPerSec(), PowerSim.getRailVoltage(), 0.02);

        encoderSim.setDistance(Units.radiansToDegrees(turretSim.getAngleRads()));
        encoderSim.setRate(Units.radiansToDegrees(turretSim.getVelocityRadPerSec()));

        PowerSim.addCurrentDraw(turretSim.getCurrentDrawAmps());
        turretLigament.setAngle(Units.radiansToDegrees(turretSim.getAngleRads()));
    }
}
