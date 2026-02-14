package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class SwerveConstants {

    // Robot Physical Properties.
    public static final Mass kRobotMass = Pounds.of(115.0);
    public static final MomentOfInertia kRobotMOI = KilogramSquareMeters.of(6.490);

    private static final Distance kWheelRadius = Inches.of(1.91);
    public static final double kWheelCOF = 1.2;

    /** Current at which the wheels start to slip. */
    private static final Current kSlipCurrent = Amps.of(120.0);

    /** Theoretical Maximum Speed at 12V. */
    public static final LinearVelocity kLinearVelocity = MetersPerSecond.of(5.5);

    // Currently only used for PathPlanner.
    public static final LinearAcceleration kLinearAcceleration = MetersPerSecondPerSecond.of(5.0);
    public static final AngularVelocity kAngularVelocity = DegreesPerSecond.of(540.0);
    public static final AngularAcceleration kAngularAcceleration = DegreesPerSecondPerSecond.of(720.0);

    /** CANBus/CANivore that all modules are connected to. */
    public static final CANBus kCANBus = new CANBus("drivebase");

    /** Configs for the Pigeon 2; leave null to skip applying Pigeon 2 configs. */
    private static final Pigeon2Configuration kPigeonConfigs = null;

    private static final int kPigeonID = 5;

    // Gear Ratios, Geometry, & Inversions.
    private static final double kDriveGearRatio = 5.36;
    private static final double kSteerGearRatio = 18.75;

    /** How much the drive motor unintentionally turns when you rotate steering (azimuth). */
    private static final double kCoupleRatio = 0.0;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    // Closed Loop Configuration.
    private static final Slot0Configs kSteerGains = new Slot0Configs()
            .withKP(100)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.33)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs kDriveGains =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.124);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    private static final TalonFXConfiguration kDriveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration kSteerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    // Help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));

    private static final CANcoderConfiguration kEncoderInitialConfigs = new CANcoderConfiguration();

    // Simulation Parameters.
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Drivetrain & Module Constants.
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonID)
            .withPigeon2Configs(kPigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            ConstantCreator = new SwerveModuleConstantsFactory<
                            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(kWheelRadius)
                    .withSteerMotorGains(kSteerGains)
                    .withDriveMotorGains(kDriveGains)
                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kLinearVelocity)
                    .withDriveMotorType(kDriveMotorType)
                    .withSteerMotorType(kSteerMotorType)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(kDriveInitialConfigs)
                    .withSteerMotorInitialConfigs(kSteerInitialConfigs)
                    .withEncoderInitialConfigs(kEncoderInitialConfigs)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Module IDs, Offsets, & Locations.
    // Front Left Module.
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 7;
    private static final int kFrontLeftEncoderId = 1;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.31982421875);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.875);
    private static final Distance kFrontLeftYPos = Inches.of(10.875);

    // Front Right Module.
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 9;
    private static final int kFrontRightEncoderId = 2;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.2138671875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.875);
    private static final Distance kFrontRightYPos = Inches.of(-10.875);

    // Back Left Module.
    private static final int kBackLeftDriveMotorId = 10;
    private static final int kBackLeftSteerMotorId = 11;
    private static final int kBackLeftEncoderId = 3;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.1611328125);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.875);
    private static final Distance kBackLeftYPos = Inches.of(10.875);

    // Back Right Module.
    private static final int kBackRightDriveMotorId = 12;
    private static final int kBackRightSteerMotorId = 13;
    private static final int kBackRightEncoderId = 4;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.139892578125);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.875);
    private static final Distance kBackRightYPos = Inches.of(-10.875);

    // Module Constants.
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft = ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightEncoderInverted);

    // Derived Values & Helpers
    public static final double kOdometryFrequency =
            new CANBus(DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    public static final double kDriveBaseRadius = Math.max(
            Math.max(
                    Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                    Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
            Math.max(
                    Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                    Math.hypot(BackRight.LocationX, BackRight.LocationY)));

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
            new Translation2d(FrontRight.LocationX, FrontRight.LocationY),
            new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
            new Translation2d(BackRight.LocationX, BackRight.LocationY)
        };
    }

    // PathPlanner & Simulation Configurations.
    public static final RobotConfig kPathPlannerConfig = new RobotConfig(
            kRobotMass,
            kRobotMOI,
            new ModuleConfig(
                    kWheelRadius,
                    kLinearVelocity,
                    kWheelCOF,
                    DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio),
                    kSlipCurrent,
                    1),
            getModuleTranslations());

    public static final DriveTrainSimulationConfig kMapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(kRobotMass)
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    // Must be a Falcon motor for this version of MapleSim.
                    // MapleMotorSim should implement DCMotorSim in upcoming versions.
                    DCMotor.getKrakenX60(1),
                    kDriveGearRatio,
                    kSteerGearRatio,
                    kDriveFrictionVoltage,
                    kSteerFrictionVoltage,
                    kWheelRadius,
                    kSteerInertia,
                    kWheelCOF));
}
