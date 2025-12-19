package igknighters.subsystems.stem.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kStem;
import igknighters.constants.ConstValues.kStem.kPivot;
import igknighters.constants.HardwareIndex.StemHW;
import igknighters.util.logging.BootupLogger;
import igknighters.util.logging.FaultManager;

public class PivotReal extends Pivot {

    /** Right */
    private final TalonFX leaderMotor;

    /** Left */
    private final TalonFX followerMotor;

    private final Pigeon2 gyro;

    // private final StatusSignal<Double> motorRots, motorVelo, leaderMotorVolts,
    // followerMotorVolts;
    private final StatusSignal<Angle> motorRots;
    private final StatusSignal<AngularVelocity> motorVelo;
    private final StatusSignal<Voltage> leaderMotorVolts, followerMotorVolts;
    private final StatusSignal<Current> leaderMotorAmps, followerMotorAmps;
    private final StatusSignal<Angle> gyroMeasurement;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final MotionMagicVoltage controlReqMotionMagic =
            new MotionMagicVoltage(0.0).withUpdateFreqHz(0);

    private boolean homedThisCycle = false;
    private boolean hasBeenEnabled = false;

    private double mechRadiansToMotorRots(Double mechRads) {
        return Units.radiansToRotations(Math.PI - mechRads) * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechRadians(Double motorRots) {
        return Math.PI - Units.rotationsToRadians(motorRots / kPivot.MOTOR_TO_MECHANISM_RATIO);
    }

    public PivotReal() {
        super(0.0);
        gyro = new Pigeon2(kPivot.PIGEON_ID, kStem.CANBUS);
        gyroMeasurement = gyro.getPitch();

        leaderMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID, kStem.CANBUS);
        followerMotor = new TalonFX(kPivot.LEFT_MOTOR_ID, kStem.CANBUS);

        // CANRetrier.retryStatusCodeFatal(
        //         () -> leaderMotor.getConfigurator().apply(getMotorConfig(true)), 10);
        // CANRetrier.retryStatusCodeFatal(
        //         () -> followerMotor.getConfigurator().apply(getMotorConfig(false)), 10);
        // CANRetrier.retryStatusCodeFatal(
        //         () -> followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true)),
        // 10);

        leaderMotor.getConfigurator().apply(getMotorConfig(true));
        followerMotor.getConfigurator().apply(getMotorConfig(false));
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

        double startingRads = Units.degreesToRadians(gyroMeasurement.getValueAsDouble());
        super.gyroRadians = startingRads;
        super.radians = startingRads;
        super.targetRadians = startingRads;

        home();

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        followerMotorAmps = leaderMotor.getTorqueCurrent();
        leaderMotorAmps = followerMotor.getTorqueCurrent();
        leaderMotorVolts = leaderMotor.getMotorVoltage();
        followerMotorVolts = followerMotor.getMotorVoltage();
        forwardLimitSwitch = leaderMotor.getForwardLimit();
        reverseLimitSwitch = leaderMotor.getReverseLimit();

        // CANSignalManager.registerSignals(
        //         kStem.CANBUS,
        //         motorRots,
        //         motorVelo,
        //         leaderMotorVolts,
        //         followerMotorVolts,
        //         leaderMotorAmps,
        //         followerMotorAmps,
        //         forwardLimitSwitch,
        //         reverseLimitSwitch,
        //         gyroMeasurement);
        motorRots.setUpdateFrequency(10);
        motorVelo.setUpdateFrequency(10);
        leaderMotorVolts.setUpdateFrequency(10);
        followerMotorVolts.setUpdateFrequency(10);
        leaderMotorAmps.setUpdateFrequency(10);
        followerMotorAmps.setUpdateFrequency(10);
        forwardLimitSwitch.setUpdateFrequency(250);
        reverseLimitSwitch.setUpdateFrequency(250);
        gyroMeasurement.setUpdateFrequency(10);

        // gyro.optimizeBusUtilization(50, 1.0);
        // leaderMotor.optimizeBusUtilization(50, 1.0);
        // followerMotor.optimizeBusUtilization(50, 1.0);

        BootupLogger.bootupLog("    Pivot initialized (real)");
    }

    private TalonFXConfiguration getMotorConfig(boolean leader) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kPivot.MOTOR_kP;
        cfg.Slot0.kI = kPivot.MOTOR_kI;
        cfg.Slot0.kD = kPivot.MOTOR_kD;
        cfg.Slot0.kS = kPivot.MOTOR_kS;
        cfg.Slot0.kV = kPivot.MOTOR_kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted =
                kPivot.INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        cfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        if (leader) {
            cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        } else {
            cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        cfg.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        return cfg;
    }

    @Override
    public void gotoPosition(double radians) {
        super.targetRadians = radians;
        this.leaderMotor.setControl(
                controlReqMotionMagic.withPosition(mechRadiansToMotorRots(radians)));
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadians = 0.0;
        this.leaderMotor.setControl(controlReqVolts.withOutput(volts));
    }

    private double getPivotRadiansPigeon() {
        return super.gyroRadians;
    }

    @Override
    public void home() {
        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()), 0.01);
        super.radians = getPivotRadiansPigeon();
        homedThisCycle = true;
    }

    private boolean isCoasting = false;

    @Override
    public void setCoast(boolean shouldBeCoasting) {
        if (shouldBeCoasting == isCoasting) return;
        isCoasting = shouldBeCoasting;
        this.followerMotor.setNeutralMode(
                shouldBeCoasting ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        this.leaderMotor.setNeutralMode(
                shouldBeCoasting ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                motorRots,
                motorVelo,
                leaderMotorVolts,
                leaderMotorAmps,
                forwardLimitSwitch,
                reverseLimitSwitch,
                followerMotorAmps,
                followerMotorVolts,
                gyroMeasurement);

        FaultManager.captureFault(
                StemHW.LeaderMotor,
                motorRots,
                motorVelo,
                leaderMotorVolts,
                leaderMotorAmps,
                forwardLimitSwitch,
                reverseLimitSwitch);

        FaultManager.captureFault(StemHW.FollowerMotor, followerMotorAmps, followerMotorVolts);

        FaultManager.captureFault(StemHW.Pigeon2, gyroMeasurement);

        super.radians = motorRotsToMechRadians(motorRots.getValueAsDouble());
        super.radiansPerSecond =
                -Units.rotationsToRadians(motorVelo.getValueAsDouble())
                        / kPivot.MOTOR_TO_MECHANISM_RATIO;
        super.leftVolts = leaderMotorVolts.getValueAsDouble();
        super.rightVolts = followerMotorVolts.getValueAsDouble();
        super.leftAmps = leaderMotorAmps.getValueAsDouble();
        super.rightAmps = followerMotorAmps.getValueAsDouble();

        super.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.Open;
        super.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.Open;

        double newGyroRadians = Units.degreesToRadians(gyroMeasurement.getValueAsDouble() + 90);

        super.gyroRadiansPerSecondAbs =
                Math.abs(super.gyroRadians - newGyroRadians) / ConstValues.PERIODIC_TIME;
        super.gyroRadians = newGyroRadians;

        if (DriverStation.isEnabled()) {
            hasBeenEnabled = true;
        }

        if (Math.abs(super.radiansPerSecond) < 0.01
                && Math.abs(super.gyroRadiansPerSecondAbs) < 0.01
                && (Units.radiansToDegrees(super.gyroRadians) > 20.0 || !hasBeenEnabled)
                && DriverStation.isDisabled()) {
            if (Math.abs(super.radians - getPivotRadiansPigeon()) > 0.05) {
                home();
            }
        }

        DogLog.log("Subsystems/Stem/Pivot/PivotSeededPivot", homedThisCycle);
        homedThisCycle = false;
        DogLog.log("Subsystems/Stem/Pivot/radians", radians);
        DogLog.log("Subsystems/Stem/Pivot/targetRadians", targetRadians);
        DogLog.log("Subsystems/Stem/Pivot/radiansPerSecond", radiansPerSecond);
        DogLog.log("Subsystems/Stem/Pivot/leftVolts", leftVolts);
        DogLog.log("Subsystems/Stem/Pivot/rightVolts", rightVolts);
        DogLog.log("Subsystems/Stem/Pivot/leftAmps", leftAmps);
        DogLog.log("Subsystems/Stem/Pivot/rightAmps", rightAmps);
        DogLog.log("Subsystems/Stem/Pivot/gyroRadians", gyroRadians);
        DogLog.log("Subsystems/Stem/Pivot/gyroRadiansPerSecondAbs", gyroRadiansPerSecondAbs);
        DogLog.log("Subsystems/Stem/Pivot/isLimitFwdSwitchHit", isLimitFwdSwitchHit);
        DogLog.log("Subsystems/Stem/Pivot/isLimitRevSwitchHit", isLimitRevSwitchHit);
    }
}
