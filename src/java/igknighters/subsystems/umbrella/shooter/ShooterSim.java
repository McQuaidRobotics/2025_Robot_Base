package igknighters.subsystems.umbrella.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import igknighters.constants.ConstValues;

public class ShooterSim extends Shooter {
    private final FlywheelSim flyWheelSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getFalcon500(1),
                            .3,
                            ConstValues.kUmbrella.kIntake.UPPER_MECHANISM_RATIO),
                    DCMotor.getFalcon500(1));

    private double voltageInput = 0.0;

    public ShooterSim() {
        super();
    }

    @Override
    public double getSpeed() {
        return 0.0;
    }

    @Override
    public void setVoltageOut(double voltage) {
        // Do nothing in simulation
        voltageInput = voltage;
    }

    @Override
    public double getTargetSpeed() {
        return 0.0;
    }

    @Override
    public void setSpeed(double radiansPerSecond) {
        // Do nothing in simulation
    }

    @Override
    public void periodic() {
        // Update the simulation
        flyWheelSim.setInput();
        flyWheelSim.update(0.02); // Assuming a 20ms periodic update
    }
}
