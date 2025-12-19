package igknighters.subsystems.umbrella.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
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
    private double speed = 0.0;
    private double voltageInput = 0.0;
    private boolean positionControled = false;

    private final ProfiledPIDController pidController =
            new ProfiledPIDController(
                    ConstValues.kUmbrella.kShooter.MOTOR_LEFT_kP,
                    ConstValues.kUmbrella.kShooter.MOTOR_LEFT_kI,
                    ConstValues.kUmbrella.kShooter.MOTOR_LEFT_kD,
                    new Constraints(ConstValues.kUmbrella.kShooter.MAX_SHOOT_SPEED, 2));

    public ShooterSim() {
        super();
    }

    @Override
    public double getSpeed() {
        return speed;
    }

    @Override
    public void setVoltageOut(double voltage) {
        // Do nothing in simulation
        positionControled = false;
        voltageInput = voltage;
    }

    @Override
    public double getTargetSpeed() {
        return 0.0;
    }

    @Override
    public void setSpeed(double radiansPerSecond) {
        DogLog.log("Subsystems/Umbrella/Shooter/Set Speed", radiansPerSecond);
        positionControled = true;
        pidController.setGoal(radiansPerSecond);
    }

    @Override
    public void periodic() {
        // Update the simulation
        if (positionControled) {
            voltageInput = pidController.calculate(speed);
        }
        flyWheelSim.setInput(voltageInput);

        speed = flyWheelSim.getAngularVelocity().in(Units.RotationsPerSecond);

        DogLog.log("Subsystems/Umbrella/Shooter/SimSpeed", speed);
        DogLog.log("Subsystems/Umbrella/Shooter/Goal Speed", pidController.getGoal().position);

        flyWheelSim.update(0.02); // Assuming a 20ms periodic update
        voltageInput = 0.0;
        positionControled = false;
    }
}
