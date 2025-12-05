package igknighters.subsystems.umbrella.intake;

import dev.doglog.DogLog;
import igknighters.subsystems.Component;

public abstract class Intake extends Component {
    protected boolean exitBeamBroken = false;
    protected double radiansPerSecondUpper = 0.0;
    protected double voltsUpper = 0.0;
    protected double ampsUpper = 0.0;
    protected double radiansPerSecondLower = 0.0;
    protected double voltsLower = 0.0;
    protected double ampsLower = 0.0;

    @Override
    public String getOverrideName() {
        return "Intake";
    }

    /**
     * @return If the exit beam is broken
     */
    public abstract boolean isExitBeamBroken();

    /**
     * @return The output of the {@code Intake} in volts
     */
    public double getVoltageOut() {
        return voltsLower;
    }

    /**
     * Runs the mechanism in open loop at the specified voltage
     *
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    /**
     * Runs the mechanism in open loop at the specified voltage
     *
     * @param volts The specified volts: [-12.0 .. 12.0]
     * @param force If the mechanism should force past the limit switches
     */
    public abstract void setVoltageOut(double volts, boolean force);

    @Override
    public void periodic() {
        DogLog.log("Subsystems/Umbrella/Intake/Exit Beam Broken", exitBeamBroken);
        DogLog.log("Subsystems/Umbrella/Intake/Rads per Second Upper", radiansPerSecondUpper);
        DogLog.log("Subsystems/Umbrella/Intake/voltsUpper", voltsUpper);
        DogLog.log("Subsystems/Umbrella/Intake/ampsUpper", ampsUpper);
        DogLog.log("Subsystems/Umbrella/Intake/radiansPerSecondLower", radiansPerSecondLower);
        DogLog.log("Subsystems/Umbrella/Intake/voltsLower", voltsLower);
        DogLog.log("Subsystems/Umbrella/Intake/ampsLower", ampsLower);
    }
}
