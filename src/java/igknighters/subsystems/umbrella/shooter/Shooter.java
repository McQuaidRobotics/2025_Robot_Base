package igknighters.subsystems.umbrella.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import igknighters.subsystems.Component;

public abstract class Shooter extends Component {

    protected double radiansPerSecondRight = 0.0;
    protected double targetRadiansPerSecondRight = 0.0;
    protected double radiansPerSecondLeft = 0.0;
    protected double targetRadiansPerSecondLeft = 0.0;
    protected double voltsRight = 0.0;
    protected double voltsLeft = 0.0;
    protected double ampsRight = 0.0;
    protected double ampsLeft = 0.0;
    protected double tempRight = 0.0;
    protected double tempLeft = 0.0;

    protected double shooterRightRPM =
            Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondRight);

    protected double shooterLeftRPM =
            Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondLeft);

    protected double targetShooterRightRPM =
            Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondRight);

    protected double targetShooterLeftRPM =
            Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondLeft);

    @Override
    public String getOverrideName() {
        return "Shooter";
    }

    /**
     * @return The rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getSpeed();

    /**
     * @return The target rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getTargetSpeed();

    /**
     * Runs the {@code Shooter} in closed loop at the specified speed
     *
     * @param speedRadPerSec The speed in Rad/S to spin the flywheel at
     */
    public abstract void setSpeed(double speedRadPerSec);

    /**
     * Runs the mechanism in open loop at the specified voltage
     *
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    /**
     * @param rpm The rpm of the shooter
     * @return
     */
    public static double rpmToMps(double rpm) {
        final double LOW_END_RPM = 4500.0;
        final double LOW_END_MPS = 11.7;
        final double HIGH_END_RPM = 8000.0;
        final double HIGH_END_MPS = 15.2;
        final double DIFF_RPM = HIGH_END_RPM - LOW_END_RPM;
        final double DIFF_MPS = HIGH_END_MPS - LOW_END_MPS;

        double clamped = MathUtil.clamp(rpm, LOW_END_RPM, HIGH_END_RPM);
        double t = (clamped - LOW_END_RPM) / DIFF_RPM;
        return (t * DIFF_MPS) + LOW_END_MPS;
    }

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Umbrella/Shooter/Rads per Second Right", radiansPerSecondRight);
        DogLog.log(
                "Subsystems/Umbrella/Shooter/Target Rads per Second Right",
                targetRadiansPerSecondRight);
        DogLog.log("Subsystems/Umbrella/Shooter/Rads per Second Left", radiansPerSecondLeft);
        DogLog.log(
                "Subsystems/Umbrella/Shooter/Target Rads per Second Left",
                targetRadiansPerSecondLeft);
        DogLog.log("Subsystems/Umbrella/Shooter/Volts Right", voltsRight);
        DogLog.log("Subsystems/Umbrella/Shooter/Volts Left", voltsLeft);
        DogLog.log("Subsystems/Umbrella/Shooter/Amps Right", ampsRight);
        DogLog.log("Subsystems/Umbrella/Shooter/Amps Left", ampsLeft);
        DogLog.log("Subsystems/Umbrella/Shooter/Temp Right", tempRight);
        DogLog.log("Subsystems/Umbrella/Shooter/Temp Left", tempLeft);
        DogLog.log("Subsystems/Umbrella/Shooter/Shooter Right RPM", shooterRightRPM);
        DogLog.log("Subsystems/Umbrella/Shooter/Shooter Left RPM", shooterLeftRPM);
        DogLog.log("Subsystems/Umbrella/Shooter/Shooter Right Target RPM", targetShooterRightRPM);
        DogLog.log("Subsystems/Umbrella/Shooter/Shooter Left Target RPM", targetShooterLeftRPM);
    }
}
