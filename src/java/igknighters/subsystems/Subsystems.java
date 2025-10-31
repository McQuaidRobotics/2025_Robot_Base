package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.LimeLightVision.LimeLights;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
  public final CommandSwerveDrivetrain swerve;
  public final LimeLights vision;
  public final Led led;
  public final ExclusiveSubsystem[] lockedResources;
  public final SharedSubsystem[] locklessResources;

  public Subsystems(CommandSwerveDrivetrain drivetrain, LimeLights vision, Led led) {
    this.swerve = drivetrain;
    this.vision = vision;
    this.led = led;
    this.lockedResources = new ExclusiveSubsystem[] {this.swerve, led};
    this.locklessResources = new SharedSubsystem[] {vision};
  }

  public static interface ExclusiveSubsystem extends Subsystem {}

  public static interface SharedSubsystem {
    default void periodic() {}

    default void simulationPeriodic() {}

    default String getName() {
      return this.getClass().getSimpleName();
    }
  }
}
