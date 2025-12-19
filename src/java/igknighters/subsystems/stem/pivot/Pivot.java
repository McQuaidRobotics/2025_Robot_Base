package igknighters.subsystems.stem.pivot;

import dev.doglog.DogLog;
import igknighters.constants.ConstValues;
import igknighters.subsystems.Component;
import igknighters.subsystems.stem.Stem;

/**
 * A standardized interface for a {@link Pivot} {@link Stem} {@link Component}. This allows us to
 * abstract over the underlying implementation of the pivot for various reasons.
 */
public abstract class Pivot extends Component {
    protected double radians;
    protected double targetRadians;
    protected double radiansPerSecond = 0.0;
    protected double leftVolts = 0.0;
    protected double rightVolts = 0.0;
    protected double leftAmps = 0.0;
    protected double rightAmps = 0.0;
    protected double gyroRadians = 0.0;
    protected double gyroRadiansPerSecondAbs = 0.0;
    protected boolean isLimitFwdSwitchHit = false;
    protected boolean isLimitRevSwitchHit = false;

    protected Pivot(double startingRadians) {
        this.radians = startingRadians;
        this.targetRadians = startingRadians;
    }

    @Override
    public String getOverrideName() {
        return "Pivot";
    }

    /**
     * Commands the pivot to move towards a certain angle in radians.
     *
     * @param radians The target angle to move to
     */
    public abstract void gotoPosition(double radians);

    /**
     * @return The current angle of the mechanism
     */
    public double getPosition() {
        DogLog.log("Subsystems/Stem/Pivot/Returning Position", this.radians);
        return this.radians;
    }

    /**
     * @return The current velocity of the mechanism in radians per second
     */
    public double getVelocity() {
        return this.radiansPerSecond;
    }

    /** Homes the pivot based on an absolute sensor */
    public abstract void home();

    /**
     * Move the pivot to the target and returns if it has reached the target. Meant to be used in a
     * kind of polling loop to wait the mechanism to reach the target.
     *
     * @param radians The target angle to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult means more
     *     tolerance
     * @return If the mechanism has reached the target
     */
    public boolean target(double radians, double tolerancMult) {
        this.gotoPosition(radians);
        return isAt(radians, tolerancMult);
    }

    /**
     * Returns if the mechanism is within a tolerance of a certain angle.
     *
     * @param radians The angle to check against
     * @param toleranceMult The multiplier to apply to the tolerance, higher mult means more
     *     tolerance
     * @return If the mechanism is within the tolerance of the angle
     */
    public boolean isAt(double radians, double toleranceMult) {
        return Math.abs(this.getPosition() - radians)
                < ConstValues.kStem.kPivot.TARGET_TOLERANCE * toleranceMult;
    }

    /**
     * Sets the mechanism to coast or brake mode.
     *
     * @param shouldBeCoasting If the mechanism should be coasting
     */
    public void setCoast(boolean shouldBeCoasting) {
        // provide a no-op implementation to make sim classes less verbose
    }

    /**
     * Runs the mechanism in open loop at the specified voltage
     *
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    @Override
    public void periodic() {
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
